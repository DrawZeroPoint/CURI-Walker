function [q, qd, qdd] = getSwingFootTraj(footpos0, footpos1, swingheight, timestp0, timestpf, Ts)
% Returns cubic polynomial trajectory for the robot's swing foot
%
% Copyright 2019 The MathWorks, Inc.

% Trajectory for X and Y
%waypointsXY = [footpos0(1:2) footpos1(1:2)];
%timestampsXY = [timestp0 timestpf];
%timevecswingXY = timestampsXY(1):Ts:timestampsXY(end);
%[XYq, XYqd, XYqdd, ~] = cubicpolytraj(waypointsXY, timestampsXY, timevecswingXY);

% Trajectory for Z

%waypointsZ = [footpos0(3) footpos0(3)+(swingheight*0.5) footpos0(3)+swingheight footpos0(3)+(swingheight*0.5) footpos0(3)];
%
%waypoints = [footpos0(1) footpos0(2) footpos0(3);
%             footpos0(1) footpos0(2) footpos0(3)+(swingheight*0.5);
%             (footpos0(1)+footpos1(1))*0.5 (footpos0(2)+footpos1(2))*0.5 footpos0(3)+(swingheight);
%             footpos1(1) footpos1(2) footpos0(3)+(swingheight*0.5);
%             footpos1(1) footpos1(2) footpos1(3);
%             ]
%         

aux = footpos1(2)-footpos0(2)
waypoints = [footpos0(1) footpos0(1) (footpos0(1)+footpos1(1))*0.5 footpos1(1) footpos1(1);
             footpos0(2) footpos0(2)+(aux*0.10) footpos0(2)+(aux*0.5) footpos0(2)+(aux*0.90) footpos1(2);
             footpos0(3) footpos0(3)+(swingheight*0.5) footpos0(3)+(swingheight) footpos0(3)+(swingheight*0.5) footpos1(3)
             ]

runtime = timestpf-timestp0
frstquarter = timestp0+(runtime*0.25)
mid = timestp0+(runtime*0.50)
thirdquarter = timestp0+(runtime*0.75)
% timestpMid = (timestp0+timestpf)*0.5 ; %2/4 Top of swing at midpoint
% timestpquarter = timestpMid*0.5; %1/4 Top of swing at midpoint

timestampsZ = [timestp0 frstquarter mid thirdquarter timestpf];
timevecswingZ = timestampsZ(1):Ts:timestampsZ(end);

[q, qd, qdd, ~] = quinticpolytraj(waypoints, timestampsZ, timevecswingZ);
%[q, qd, qdd, ~] = bsplinepolytraj(waypoints, timestampsZ, timevecswingZ);

% combine xy and z trajectory
% q = [XYq; Zq];
% qd = [XYqd; Zqd];
% qdd = [XYqdd; Zqdd];

end