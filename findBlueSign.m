function [signRel, found] = findBlueSign(rgb, depth, cam)
%#codegen
% FINDBLUESIGN  Finds the blue accessibility sign and says where it is.
% Uses plain MATLAB only (no Image Processing Toolbox needed).
%   rgb    colour image (H x W x 3)
%   depth  depth image (H x W), aligned with the colour image
%   cam    camera settings from qcarCamera()
%   signRel  [forward left] position of the sign relative to the car (m),
%            [NaN NaN] if no sign is found
%   found    true when a sign was found

% ---- Settings (tune with the Color Thresholder app or by trial) ----
H_MIN = 0.55;  H_MAX = 0.70;   % blue hue range (MATLAB hue runs 0 to 1)
S_MIN = 0.45;  V_MIN = 0.25;   % ignore washed-out and dark pixels
MIN_AREA = 150;                % ignore blobs smaller than this (pixels)
ASPECT_MIN = 0.7;  ASPECT_MAX = 1.4;   % the sign is roughly square
GLYPH_MIN = 0.05;  GLYPH_MAX = 0.45;   % share of white pixels (the wheelchair symbol)

signRel = [NaN NaN];
found = false;

% 1. Colour mask: which pixels are "sign blue"?
hsv = rgb2hsv(rgb);
H = hsv(:,:,1);  S = hsv(:,:,2);  V = hsv(:,:,3);
blue = H >= H_MIN & H <= H_MAX & S >= S_MIN & V >= V_MIN;

% 2. Clean it: remove specks (opening), then close small gaps (closing).
blue = dilateSq(erodeSq(blue, 3), 3);
blue = erodeSq(dilateSq(blue, 7), 7);

% 3. Split the mask into separate blobs, then check each one's size,
%    shape and white symbol. Keep the biggest valid blob.
[labels, nBlobs] = labelBlobs(blue);
bestArea = 0;  bestBox = zeros(1, 4);   % [r1 r2 c1 c2]
for k = 1:nBlobs
    [rows, cols] = find(labels == k);
    a = numel(rows);
    if a < MIN_AREA
        continue
    end
    r1 = min(rows); r2 = max(rows); c1 = min(cols); c2 = max(cols);
    aspect = (c2 - c1 + 1) / (r2 - r1 + 1);
    if aspect < ASPECT_MIN || aspect > ASPECT_MAX
        continue
    end
    white = S(r1:r2, c1:c2) < 0.25 & V(r1:r2, c1:c2) > 0.7;
    glyph = mean(white(:));
    if glyph < GLYPH_MIN || glyph > GLYPH_MAX
        continue
    end
    if a > bestArea
        bestArea = a;  bestBox = [r1 r2 c1 c2];
    end
end
if bestArea == 0
    return
end

% 4. Distance: median depth of the sign's blue pixels.
r1 = bestBox(1); r2 = bestBox(2); c1 = bestBox(3); c2 = bestBox(4);
Z = double(depth(r1:r2, c1:c2)) * cam.depthScale;
m = blue(r1:r2, c1:c2) & Z > 0 & isfinite(Z);
if ~any(m(:))
    return
end
Z0 = median(Z(m));

% 5. Centre pixel + distance -> position relative to the car.
u = (c1 + c2) / 2;
v = (r1 + r2) / 2;
p = pixelToCar(u, v, Z0, cam);
signRel = [p(1) p(2)];
found = true;
end

%% ---------------- Helpers (plain MATLAB) ----------------
function out = erodeSq(mask, k)
% A pixel stays true only if its whole k x k neighbourhood is true.
out = conv2(double(mask), ones(k), 'same') >= k*k - 0.5;
end

function out = dilateSq(mask, k)
% A pixel becomes true if anything in its k x k neighbourhood is true.
out = conv2(double(mask), ones(k), 'same') > 0.5;
end

function [labels, n] = labelBlobs(mask)
% Gives each connected group of true pixels its own number (4-connected).
[nr, nc] = size(mask);
labels = zeros(nr, nc);
queue = zeros(nr*nc, 1);
n = 0;
for start = 1:nr*nc
    if mask(start) && labels(start) == 0
        n = n + 1;
        labels(start) = n;
        head = 1;  tail = 1;  queue(1) = start;
        while head <= tail
            idx = queue(head);  head = head + 1;
            r = mod(idx - 1, nr) + 1;  c = floor((idx - 1) / nr) + 1;
            % up, down, left, right neighbours
            if r > 1,  j = idx - 1;  if mask(j) && labels(j) == 0, labels(j) = n; tail = tail + 1; queue(tail) = j; end, end
            if r < nr, j = idx + 1;  if mask(j) && labels(j) == 0, labels(j) = n; tail = tail + 1; queue(tail) = j; end, end
            if c > 1,  j = idx - nr; if mask(j) && labels(j) == 0, labels(j) = n; tail = tail + 1; queue(tail) = j; end, end
            if c < nc, j = idx + nr; if mask(j) && labels(j) == 0, labels(j) = n; tail = tail + 1; queue(tail) = j; end, end
        end
    end
end
end

function p = pixelToCar(u, v, Z, cam)
% Pixel (u, v) with depth Z  ->  [forward left height] relative to the car
xc = (u - cam.cx) * Z / cam.fx;          % right of the camera
yc = (v - cam.cy) * Z / cam.fy;          % below the camera
cp = cos(cam.pitch);  sp = sin(cam.pitch);
fwd = Z*cp - yc*sp + cam.forwardOffset;
down = Z*sp + yc*cp;
p = [fwd, -xc, cam.height - down];
end