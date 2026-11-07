function [signRel, found] = findBlueSign(rgb, depth, cam)
%#codegen
% FINDBLUESIGN  Finds the blue accessibility sign and says where it is.
%   rgb    colour image (H x W x 3)
%   depth  depth image (H x W), aligned with the colour image
%   cam    camera settings from qcarCamera()
%   signRel  [forward left] position of the sign relative to the car (m),
%            [NaN NaN] if no sign is found
%   found    true when a sign was found

% ---- Settings (tune with the Color Thresholder app) ----
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

% 2. Clean it: remove specks, then fill the holes left by the white symbol.
blue = imopen(blue, strel('square', 3));
blue = imclose(blue, strel('square', 7));

% 3. Check every blob's size, shape and white symbol; keep the biggest valid one.
stats = regionprops(blue, 'Area', 'BoundingBox');
bestArea = 0;  bestBox = zeros(1, 4);
[nr, nc] = size(blue);
for k = 1:numel(stats)
    a = stats(k).Area;  bb = stats(k).BoundingBox;
    aspect = bb(3) / bb(4);
    if a < MIN_AREA || aspect < ASPECT_MIN || aspect > ASPECT_MAX
        continue
    end
    [r1, r2, c1, c2] = boxRows(bb, nr, nc);
    white = S(r1:r2, c1:c2) < 0.25 & V(r1:r2, c1:c2) > 0.7;
    glyph = mean(white(:));
    if glyph < GLYPH_MIN || glyph > GLYPH_MAX
        continue
    end
    if a > bestArea
        bestArea = a;  bestBox = bb;
    end
end
if bestArea == 0
    return
end

% 4. Distance: median depth of the sign's pixels (median ignores noisy pixels).
[r1, r2, c1, c2] = boxRows(bestBox, nr, nc);
Z = double(depth(r1:r2, c1:c2)) * cam.depthScale;
m = blue(r1:r2, c1:c2) & Z > 0 & isfinite(Z);
if ~any(m(:))
    return
end
Z0 = median(Z(m));

% 5. Turn the sign's centre pixel + distance into a position relative to the car.
u = bestBox(1) + bestBox(3)/2;
v = bestBox(2) + bestBox(4)/2;
p = pixelToCar(u, v, Z0, cam);
signRel = [p(1) p(2)];
found = true;
end

function [r1, r2, c1, c2] = boxRows(bb, nr, nc)
r1 = max(1, round(bb(2)));  r2 = min(nr, round(bb(2) + bb(4)));
c1 = max(1, round(bb(1)));  c2 = min(nc, round(bb(1) + bb(3)));
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