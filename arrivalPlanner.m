function [desiredSpeed, state, reasonCode, target] = arrivalPlanner(t, pose, speed, stopSignDist, blueSignRel, rampRel, obstacleDist, doorClear)
%#codegen
% ARRIVALPLANNER  Decides how the taxi arrives at, waits at, and leaves every stop.
%
%   Inputs
%     t             current time (s)
%     pose          [x y heading] in metres and radians (from localization)
%     speed         measured speed (m/s)
%     stopSignDist  distance to the stop sign (m), Inf if none is seen
%     blueSignRel   [forward left] position of an accessibility sign relative
%                   to the car (m), [NaN NaN] if none is seen   (from findBlueSign)
%     rampRel       [forward left] position of a curb ramp relative to the car (m),
%                   [NaN NaN] if none is seen                  (from findRamp)
%     obstacleDist  distance to an obstacle ahead (m), Inf if none
%     doorClear     true when the area beside the door is free
%
%   Outputs
%     desiredSpeed  speed command for speedController (m/s)
%     state         1 CRUISE, 2 APPROACH, 3 DOCK, 4 DWELL, 5 CHECK_CLEAR, 6 DEPART, 7 DONE, 8 SEARCH
%     reasonCode    why the car is doing what it does (see list below)
%     target        [x y] of the point the car is currently aiming to stop at

%% ---------------- Settings (edit these) ----------------
V_CRUISE  = 0.50;   % normal driving speed (m/s), same as the given model
V_SEARCH  = 0.20;   % speed while looking for the accessible spot (m/s)
V_CRAWL   = 0.10;   % slow final approach speed (m/s)
V_MIN     = 0.04;   % slowest creeping speed before the final stop (m/s)
V_CAUTION = 0.25;   % speed when the map expects a sign the camera can't see (m/s)

PROFILE    = 1;     % rider profile: 1 = gentle, 2 = standard
A_BRAKE_GENTLE   = 0.15;  A_BRAKE_STANDARD = 0.40;   % braking (m/s^2)
A_ACCEL_GENTLE   = 0.15;  A_ACCEL_STANDARD = 0.40;   % pulling away (m/s^2)

DOCK_DIST       = 0.20;   % start the slow crawl this far from the stop (m)
STOP_TOL        = 0.03;   % "close enough" to the stop point (m)
APPROACH_MARGIN = 0.30;   % extra distance to start braking early (m)

% Trip stops in order: [x  y  type  waitSeconds  zoneRadius]
%   type 1 = pick-up at an exact point
%   type 2 = drop-off zone. Inside the zone the car picks, in this order:
%            1) a curb ramp, 2) an accessibility sign, 3) the normal spot (x, y)
% Replace with real coordinates from the coordinate helper.
STOPS = [1.5  0.0  1  4.0  0.0;    % passenger 1: pick-up at house 1
         2.5  3.0  2  5.0  0.8;    % passenger 1: drop-off zone A
         0.0  1.5  1  4.0  0.0;    % passenger 2: pick-up at house 2
         2.8  0.0  2  5.0  0.8];   % passenger 2: drop-off zone B

ZONE_MARGIN = 0.30;     % a sign or ramp may sit this far outside the zone edge (m)
DOOR_OFFSET = 0.00;     % door position ahead of the car's centre (m)

SIGN_RANGE    = 1.5;    % only react to stop signs closer than this (m)
SIGN_GAP      = 0.20;   % stop this far before the sign (m)
SIGN_WAIT     = 3.0;    % full stop time at a stop sign (s)
SIGN_COOLDOWN = 4.0;    % ignore stop signs for this long after leaving one (s)
SIGN_MAP_XY   = [4.0 2.0];  % where the map says the stop line is
SKEPTIC_RADIUS = 0.8;   % "a sign should be here" zone around SIGN_MAP_XY (m)

OBS_RANGE  = 1.5;       % react to obstacles closer than this (m)
OBS_GAP    = 0.35;      % keep this much space to an obstacle (m)
STUCK_TIME = 8.0;       % ask for help after waiting this long behind an obstacle (s)

%% ---------------- Names for states and reasons ----------------
S_CRUISE = 1; S_APPROACH = 2; S_DOCK = 3; S_DWELL = 4;
S_CHECK_CLEAR = 5; S_DEPART = 6; S_DONE = 7; S_SEARCH = 8;

R_CRUISE = 0;  R_APPROACH_PICKUP = 1; R_APPROACH_DROPOFF = 2; R_ACCESSIBLE_ZONE = 3;
R_APPROACH_SIGN = 4; R_DOCKING = 5; R_WAIT_SIGN = 6; R_BOARDING = 7; R_ALIGHTING = 8;
R_DOOR_BLOCKED = 9; R_DEPARTING = 10; R_OBSTACLE = 11; R_NEED_HELP = 12;
R_SIGN_NOT_SEEN = 13; R_TRIP_DONE = 14; R_SEARCHING = 15; R_SIGN_LOCATED = 16;
R_RAMP_FOUND = 17; R_NO_RAMP = 18;

%% ---------------- Memory between calls ----------------
persistent st tEnter idx targetType targetXY cooldownUntil obsSince blueSeen
persistent zoneXY signXY signScore rampLocked
if isempty(st)
    st = S_CRUISE;  tEnter = t;  idx = 1;  targetType = 0;
    targetXY = [0 0];  cooldownUntil = -Inf;  obsSince = -1;  blueSeen = false;
    zoneXY = [0 0];  signXY = [NaN NaN];  signScore = Inf;  rampLocked = false;
end

nStops = size(STOPS, 1);
c = cos(pose(3));  s = sin(pose(3));
signSeenNow = ~any(isnan(blueSignRel));
rampSeenNow = ~any(isnan(rampRel));
if signSeenNow
    blueSeen = true;
end

% Choose braking and pull-away strength from the rider profile.
% An accessible drop-off always uses the gentle setting.
if PROFILE == 1 || blueSeen
    aBrake = A_BRAKE_GENTLE;  aAccel = A_ACCEL_GENTLE;
else
    aBrake = A_BRAKE_STANDARD; aAccel = A_ACCEL_STANDARD;
end

v = V_CRUISE;
reason = R_CRUISE;

%% ---------------- State machine ----------------
switch st
    case S_CRUISE
        v = V_CRUISE;  reason = R_CRUISE;
        startBrakingAt = V_CRUISE^2 / (2*aBrake) + DOCK_DIST + APPROACH_MARGIN;

        if idx <= nStops
            dTrip = hypot(pose(1) - STOPS(idx,1), pose(2) - STOPS(idx,2));
            if STOPS(idx,3) == 2
                % Accessible drop-off zone: start searching before the zone edge,
                % early enough to slow from cruise to search speed gently.
                searchLead = (V_CRUISE^2 - V_SEARCH^2) / (2*aBrake);
                if dTrip <= STOPS(idx,5) + searchLead
                    st = S_SEARCH;  tEnter = t;  targetType = 2;
                    zoneXY = [STOPS(idx,1) STOPS(idx,2)];
                    targetXY = zoneXY;
                    signXY = [NaN NaN];  signScore = Inf;  rampLocked = false;
                end
            elseif dTrip <= startBrakingAt
                st = S_APPROACH;  tEnter = t;
                targetType = STOPS(idx,3);
                targetXY = [STOPS(idx,1) STOPS(idx,2)];
            end
        end

        % Stop sign seen? Lock its stop point onto the map.
        if st == S_CRUISE && stopSignDist < SIGN_RANGE && t >= cooldownUntil
            d = max(stopSignDist - SIGN_GAP, 0);
            targetXY = [pose(1) + d*c, pose(2) + d*s];
            targetType = 3;
            st = S_APPROACH;  tEnter = t;
        end

        % Skeptic: the map says a sign is just ahead but the camera doesn't see it.
        toSignX = SIGN_MAP_XY(1) - pose(1);  toSignY = SIGN_MAP_XY(2) - pose(2);
        signAhead = toSignX*c + toSignY*s > 0;
        if st == S_CRUISE && t >= cooldownUntil && signAhead && ...
                hypot(toSignX, toSignY) < SKEPTIC_RADIUS
            v = V_CAUTION;  reason = R_SIGN_NOT_SEEN;
        end

    case S_SEARCH
        % Head for the normal drop-off spot, but switch to a better spot if one
        % appears in the zone: ramp first, then accessibility sign.
        R = STOPS(idx,5);

        % Accessibility sign: keep the one nearest the zone centre.
        if signSeenNow && ~rampLocked
            p = relToMap(pose, blueSignRel);
            score = hypot(p(1) - zoneXY(1), p(2) - zoneXY(2));
            along = blueSignRel(1) - DOOR_OFFSET;
            if score <= R + ZONE_MARGIN && score < signScore && along > DOCK_DIST + 0.05
                signXY = p;  signScore = score;
                targetXY = [pose(1) + along*c, pose(2) + along*s];   % stop level with the sign
            end
        end

        % Curb ramp in the zone: best option, lock it.
        if rampSeenNow && ~rampLocked
            p = relToMap(pose, rampRel);
            along = rampRel(1) - DOOR_OFFSET;
            if hypot(p(1) - zoneXY(1), p(2) - zoneXY(2)) <= R + ZONE_MARGIN && along > DOCK_DIST + 0.05
                rampLocked = true;
                targetXY = [pose(1) + along*c, pose(2) + along*s];   % door level with the ramp
            end
        end

        % Speed: ease down to search speed by the zone edge, then the usual
        % gentle braking curve onto whichever target is current.
        d = hypot(pose(1) - targetXY(1), pose(2) - targetXY(2));
        dZone = hypot(pose(1) - zoneXY(1), pose(2) - zoneXY(2));
        vTop = min(V_CRUISE, sqrt(V_SEARCH^2 + 2*aBrake*max(dZone - R, 0)));
        v = min(vTop, max(V_CRAWL, sqrt(2*aBrake*max(d - DOCK_DIST, 0))));
        if rampLocked
            reason = R_RAMP_FOUND;
        elseif ~isnan(signXY(1))
            reason = R_SIGN_LOCATED;
        else
            reason = R_SEARCHING;
        end
        if d <= DOCK_DIST
            st = S_DOCK;  tEnter = t;
            if ~rampLocked && isnan(signXY(1))
                reason = R_NO_RAMP;
            end
        end

    case S_APPROACH
        d = hypot(pose(1) - targetXY(1), pose(2) - targetXY(2));
        vTop = V_CRUISE;
        if targetType == 2
            vTop = V_SEARCH;    % already slowed for the drop-off zone; don't speed back up
        end
        v = min(vTop, max(V_CRAWL, sqrt(2*aBrake*max(d - DOCK_DIST, 0))));
        if targetType == 1
            reason = R_APPROACH_PICKUP;
        elseif targetType == 2
            if rampLocked
                reason = R_RAMP_FOUND;
            elseif blueSeen
                reason = R_ACCESSIBLE_ZONE;
            else
                reason = R_APPROACH_DROPOFF;
            end
        else
            reason = R_APPROACH_SIGN;
        end
        if d <= DOCK_DIST
            st = S_DOCK;  tEnter = t;
        end

    case S_DOCK
        d = hypot(pose(1) - targetXY(1), pose(2) - targetXY(2));
        v = max(V_MIN, V_CRAWL * min(1, d / DOCK_DIST));
        reason = R_DOCKING;
        if d <= STOP_TOL
            v = 0;  st = S_DWELL;  tEnter = t;
        end

    case S_DWELL
        v = 0;
        if targetType == 3
            waitTime = SIGN_WAIT;  reason = R_WAIT_SIGN;
        else
            waitTime = STOPS(idx,4);
            if targetType == 1
                reason = R_BOARDING;
            else
                reason = R_ALIGHTING;
            end
        end
        if t - tEnter >= waitTime
            tEnter = t;
            if targetType == 3
                st = S_DEPART;
                cooldownUntil = t + SIGN_COOLDOWN;
            else
                st = S_CHECK_CLEAR;
            end
        end

    case S_CHECK_CLEAR
        v = 0;
        if doorClear
            reason = R_DEPARTING;
            if targetType == 2
                blueSeen = false;
            end
            idx = idx + 1;
            tEnter = t;
            if idx > nStops
                st = S_DONE;  reason = R_TRIP_DONE;
            else
                st = S_DEPART;
            end
        else
            reason = R_DOOR_BLOCKED;
        end

    case S_DEPART
        v = min(V_CRUISE, V_CRAWL + aAccel*(t - tEnter));
        reason = R_DEPARTING;
        if v >= V_CRUISE
            st = S_CRUISE;  targetType = 0;
        end

    case S_DONE
        v = 0;  reason = R_TRIP_DONE;

    otherwise
        st = S_CRUISE;
end

%% ---------------- Obstacle ahead (applies in every moving state) ----------------
stuck = false;
if obstacleDist < OBS_RANGE && st ~= S_DONE
    vObs = sqrt(2*aBrake*max(obstacleDist - OBS_GAP, 0));
    if vObs < v
        v = vObs;  reason = R_OBSTACLE;
        if obstacleDist <= OBS_GAP + 0.05 && speed < 0.02
            stuck = true;
            if obsSince < 0
                obsSince = t;
            end
            if t - obsSince >= STUCK_TIME
                reason = R_NEED_HELP;
            end
        end
    end
end
if ~stuck
    obsSince = -1;
end

%% ---------------- Outputs ----------------
desiredSpeed = v;
state = st;
reasonCode = reason;
target = targetXY;
end

%% ---------------- Helpers ----------------
function p = relToMap(pose, rel)
% [forward left] relative to the car  ->  [x y] on the map
c = cos(pose(3));  s = sin(pose(3));
p = [pose(1) + rel(1)*c - rel(2)*s, pose(2) + rel(1)*s + rel(2)*c];
end

function rel = mapToRel(pose, p)
% [x y] on the map  ->  [forward left] relative to the car
c = cos(pose(3));  s = sin(pose(3));
dx = p(1) - pose(1);  dy = p(2) - pose(2);
rel = [dx*c + dy*s, -dx*s + dy*c];
end