function [rampRel, found, binCentres, binHeight] = findRamp(depth, cam)
%#codegen
% FINDRAMP  Looks along the curb in the depth image for a gap where the curb
% drops to road level (a curb ramp / curb cut).
%   depth  depth image (H x W)
%   cam    camera settings from qcarCamera()
%   rampRel     [forward left] of the ramp's centre relative to the car (m),
%               [NaN NaN] if none is found
%   found       true when a ramp was found
%   binCentres  forward distance of each slice (m)       } the curb's height
%   binHeight   curb-top height in each slice (m), NaN   } profile, for plotting

% ---- Settings ----
SIDE = -1;            % -1 = curb on the car's right, +1 = on its left
LAT_MIN = 0.15;       % look between these sideways distances (m)
LAT_MAX = 0.60;
FWD_MIN = 0.40;       % ...and between these distances ahead (m)
FWD_MAX = 1.60;
BIN = 0.04;           % length of each slice along the road (m)
CURB_H = 0.03;        % expected curb height (m). A real 15 cm curb is about 1.5 cm at QCar scale
MIN_PTS = 8;          % ignore slices with fewer depth points than this
COVER_LAT = 0.35;     % a slice counts only if the camera sees at least this far sideways (past the curb edge)
RAMP_MIN_LEN = 0.12;  % a ramp must be at least this long (m)
STEP = 2;             % use every 2nd pixel (faster, still plenty of points)

nb = round((FWD_MAX - FWD_MIN) / BIN);
binCentres = FWD_MIN + BIN*((1:nb) - 0.5);
binHeight = NaN(1, nb);
rampRel = [NaN NaN];
found = false;

% 1. Every depth pixel -> a 3D point in the car's frame (forward, sideways, height).
[nr, nc] = size(depth);
[U, V] = meshgrid(1:STEP:nc, 1:STEP:nr);
Z = double(depth(1:STEP:nr, 1:STEP:nc)) * cam.depthScale;
xc = (U - cam.cx) .* Z / cam.fx;
yc = (V - cam.cy) .* Z / cam.fy;
cp = cos(cam.pitch);  sp = sin(cam.pitch);
fwd = Z*cp - yc*sp + cam.forwardOffset;
h = cam.height - (Z*sp + yc*cp);         % height above the road
lat = SIDE * (-xc);                      % sideways distance toward the curb side

% 2. Keep only points in the strip along the curb.
band = Z > 0 & isfinite(Z) & lat >= LAT_MIN & lat <= LAT_MAX & ...
       fwd >= FWD_MIN & fwd < FWD_MAX;
b = floor((fwd - FWD_MIN) / BIN) + 1;

% 3. Height profile: for each slice along the road, the height of the top
%    surface (80th percentile, so the curb top wins over the road and noise).
%    A slice only counts if the camera sees far enough sideways to include the curb.
for k = 1:nb
    inBin = band & b == k;
    hk = h(inBin);
    if numel(hk) >= MIN_PTS && max(lat(inBin)) >= COVER_LAT
        hs = sort(hk);
        binHeight(k) = hs(max(1, round(0.8*numel(hs))));
    end
end

% 4. Label slices: curb (high) or low (road level). Unknown slices are neither,
%    except a single unknown slice between two low ones counts as low
%    (noise or the curb's shadow can blank out one slice).
isCurb = binHeight > 0.5*CURB_H;
isLow  = binHeight <= 0.5*CURB_H;
for k = 2:nb-1
    if isnan(binHeight(k)) && isLow(k-1) && isLow(k+1)
        isLow(k) = true;
    end
end

% 5. A ramp = a run of low slices, long enough, with curb right beside it.
bestStart = 0;  bestEnd = 0;
k = 1;
while k <= nb
    if isLow(k)
        k1 = k;
        while k <= nb && isLow(k)
            k = k + 1;
        end
        k2 = k - 1;
        longEnough = (k2 - k1 + 1) * BIN >= RAMP_MIN_LEN;
        curbBeside = (k1 > 1 && isCurb(k1 - 1)) || (k2 < nb && isCurb(k2 + 1));
        if longEnough && curbBeside && bestStart == 0
            bestStart = k1;  bestEnd = k2;   % nearest valid ramp
        end
    else
        k = k + 1;
    end
end
if bestStart == 0
    return
end

% 6. Ramp position: middle of the run ahead, typical sideways distance of its points.
fwdC = (binCentres(bestStart) + binCentres(bestEnd)) / 2;
inRamp = band & b >= bestStart & b <= bestEnd;
rampRel = [fwdC, SIDE * median(lat(inRamp))];
found = true;
end