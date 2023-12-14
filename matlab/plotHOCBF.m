%
% DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
%
% This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001.
% Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of
% the Under Secretary of Defense for Research and Engineering.
%
% © 2023 Massachusetts Institute of Technology.
%
% The software/firmware is provided to you on an As-Is basis
%
% Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014).
% Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above.
% Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
%

function outframes = plotHOCBF(data,varargin)

clf
close all

if length(varargin) >= 1
    stridelength = varargin{1};
else
    stridelength = 1;
end

make_video = false;
if length(varargin) >= 2
    filename = varargin{2};
    make_video = true;
end

state = data.state;
% state(4,:) = marine2normal_rad(state(4,:));
obstacle_pos = data.obstacle_pos;
R = data.R;
u = data.u;

num_frames = size(state,2);

M(1) = struct('cdata',[],'colormap',[]);
idx = 1;

thetavec = 0:0.1:2*pi;

n_obs = size(obstacle_pos,2);

for tt=1:stridelength:num_frames
    
    clf
    hold on

    % Plot obstacles
    for jj=1:n_obs
        plot(obstacle_pos(1,jj)+R*cos(thetavec), obstacle_pos(2,jj)+R*sin(thetavec),"r-");
    end

    % Plot agent
    plot(state(1,tt), state(2,tt),'bx');
    linex = [state(1,tt), state(1,tt)+(u(1,tt)*cos(state(4,tt)))];
    liney = [state(2,tt), state(2,tt)+(u(1,tt)*sin(state(4,tt)))];
    plot(linex, liney, 'b-');

    hold off

    xlim([-150/2,150/2]);
    ylim([-50,100]);

    axis square
    drawnow

    M(idx) = getframe;
    idx = idx+1;
end

outframes = M;

if make_video
    workingDir = "./";
    outputVideo = VideoWriter(fullfile(workingDir,filename));

    outputVideo.FrameRate = 24;

    open(outputVideo)

    for ii=2:length(M)
        try
        writeVideo(outputVideo,M(ii));
        catch
            ;
        end
    end

    close(outputVideo)

end

end