%================================= lynx6 =================================
%
%
%
%
%================================= lynx6 =================================

%
%  Name:	    lynx6.m
%
%  Author:	    Patricio A. Vela, pvela@gatech.edu
%
%  Created:	    2007/08/09
%  Modified:	2014/03/26
%
%================================= lynx6 =================================
classdef lynx6 < handle


properties
  alpha;

  alphaIds    = [   0    1    2    3    4    5];		
  				% Servo ID numbers on the SSC-32 control board.
  alpha2musec = diag(1./[0.09 0.09 0.09 0.09 0.09 0.09/75]);	
  				% Converts degrees to microsecs for manipulator.
        				%  This may be only kinda accurate since it 
        				%  assumes the same swing for all servos, which
        				%  is not necessarily true.  
  				%  I don't really use this since the musec
  				%  commands are interpolated from the xxLims
  				%  variables.
        				%  Need to investigate.
  
  alphaHome   = [   0;   0;   0;   0;   0; 1.0];		
  				% Home position as a joint configuration.
  %alphaSleep  = [   0,  75, -120, -75,   0, 1.0];	% Folded up.
  alphaSleep  = [   0;  35;   70;  70;   0;  1.0];	% Leaning on block.
  				% Sleep position as a joint configuration.
  				%   This is what you put it to before powering 
        				%   down.
  
  alphaOrient = [ -1,   1,   1,  -1,   1,  -1];		
  				% To switch orientation in case servo rotates 
        				%   in the wrong direction 
        				%   (e.g. need right-hand rule).
  
  alphaLims = [ -90, -90, -90, -90, -90, 0.00;		
                  0,   0,   0,   0,   0, 3/4;
        	       90,  90,  90,  90,  80, 1.125];
  				% Limits of the joints, either angular values 
      			%   or linear values as in the gripper.
  				%   The middle value is some measured
  				%   intermediate location.
  musecLims = [ 525  640  860  580  790 1475;			
               1460 1520 1725 1455 1580 1760;			
               2320 2400 2500 2340 2500 2500];
  				% How these limits translate to microseconds 
        				%   for the servo commands.
  
  linklen;
  serialid;
  Jak;
end

%)
%
%============================ Member Functions ===========================
%
%(

methods

  %------------------------------- lynx6 -------------------------------
  %
  %  Constructor
  %
  function this = lynx6(setup)

  mm2in   = 1/25.4;
  linklen = [110.6 120 120 130 20]*mm2in;			
  				% Measured link lengths of the manipulator in
  				%   millimeters but converted to inches.

  % Serial port setup defaults:
  serport.com  = 'COM1';		% COM port is COM1.
  serport.baud = 115200;		% Baud rate is 115200 bps.
  serport.terminator = 'CR';	% Message terminator is Carriage Return.

  % Parse the setup argument now.
  if (nargin > 0)
    fnames = fieldnames(setup);
    for ii = 1:length(fnames)
      fnames{ii}
      switch fnames{ii}
        case {'linklen','alphaHome','alphaOrient'},
          eval(['this.' fnames{ii} ' = getfield(setup, fnames{ii});']);

          case 'musecLims',
          disp('Here 2');
          if (size(setup.musecLims,2) == 6)
  	        if (size(setup.musecLims, 1) == 2)
  	          this.musecLims(:,1) = setup.musecLims(:,1);
  	          this.musecLims(:,3) = setup.musecLims(:,2);
  	          this.musecLims(:,2) = mean(setup.musecLims,1)
  	        elseif (size(setup.musecLims, 1) == 3)
  	          this.musecLims = setup.musecLims;
  	        end
          end

        case 'alphaLims'
          disp('Here 3');
          if (size(setup.alphaLims,2) == 6)
            if (size(setup.alphaLims,1) == 2)
  	          this.alphaLims(:,1) = setup.alphaLims(:,1);
  	          this.alphaLims(:,3) = setup.alphaLims(:,2);
  	          this.alphaLims(:,2) = mean(setup.alphaLims,1)
  	        elseif (size(setup.alphaLims, 1) == 3)
  	          this.alphaLims = setup.alphaLims;
  	        end
          end

        case 'COM','baud','terminator',
          eval(['serport.' fnames{ii} ' = getfield(setup, fnames{ii});']);
      end
    end
  end
  this.musecLims
  this.alphaLims
  this.alphaOrient
  % Open up the serial port.
  if (ismac)
    this.serialid = serial(serport.com,'BaudRate',serport.baud, ...
                                       'Terminator',serport.terminator);
    fopen(this.serialid);

  elseif (ispc)
    this.serialid = serial(serport.com,'BaudRate',serport.baud, ...
                                       'Terminator',serport.terminator)
    fopen(this.serialid)
    % Note that in Windows, if the program crashes then the com port is
    % still open.  You will not be able to reopen the port and you won't
    % be able to access the serialid you had before.  The only option I
    % know of is to restart Matlab.  Or you could try to do an 'fclose
    % all' and see if that works.  I haven't tested it out.
    %
    %   Read documentation: 'help fclose'
  
  elseif isunix
    device = '/dev/ttyS0'
    this.serialid = fopen(device,'w+')
  end

  end

  %----------------------------- gotoSleep -----------------------------
  %
  %  The first and second to last member function to call.  This puts
  %  the manipulator into a rest position for shutdown.  At startup, it
  %  returns the manipulator to the rest position.  This is to prevent
  %  huge jerky movements as the motor goes to the first commanded
  %  position from wherever it is located at.
  %
  function gotoSleep(this, time)
  
  if (nargin == 1)
    time = 4;
  end
  
  this.setArm(this.alphaSleep, time);
  
  end

  %------------------------------ gotoHome -----------------------------
  %
  %  This is the default position after achieving the sleep position.
  %  The home position is typically where it will go when powered up,
  %  but not being used.
  %
  function gotoHome(this, time)
  
  if (nargin == 1)
    time = 4;
  end
  
  this.setArm(this.alphaHome, time);
  
  end
  
  
  %----------------------------- setServos -----------------------------
  %
  %  Send raw motor commands to the servomotors.
  %
  function setServos(this, pwmsigs, time, defaultLims)
  
  if ((nargin < 3) || isempty(time))
    time = 2;
  end
  
  if (nargin < 4)
    defaultLims = true;
  end
    
  if ( (size(pwmsigs,1) ~= 6) || (size(pwmsigs,2) ~= 1) )
    disp('Incorrect dimensions, ignoring command [expect: 6x1 column vector].');
    return;
  end
    
  if (defaultLims)
    ticks = min(this.musecLims(3,:),max(this.musecLims(1,:), pwmsigs'));
  else
    ticks = min(2500, max(500, pwmsigs'));
  end
  
  cmdstr = sprintf('#%d P%d ',[this.alphaIds ; ticks]);		
  cmdstr = [cmdstr sprintf(' T%d \n', round(time*1000))];
  
  %fprintf(cmdstr); 			% Print actual command to STDOUT. To debug.
  fprintf(this.serialid, cmdstr);
 
  end
  
  %------------------------------- setArm ------------------------------
  %
  %  Send the arm to some position, with an optional time duration of
  %  movement.  If the gripper width is not passed, then defaults to the
  %  home position gripper width.
  %
  function setArm(this, alpha, time)

  if (nargin == 2)
    time = 4;
  end

  if ( (size(alpha,1) == 5) && (size(alpha,2) == 1) )
    alpha(6) = alphaHome(6);
  elseif ( (size(alpha,1) ~= 6) || (size(alpha,2) ~= 1) )
    disp('Incorrect dimensions, ignoring command');
    disp('    [expect: 5x1 or 6x1 column vector].');
    return;
  end

  alpha = min(this.alphaLims(3,:),max(this.alphaLims(1,:), alpha'));
  for ii = 1:length(this.alphaIds)
    if (this.alphaOrient(ii) == 1)
      ticks(ii) = interp1(this.alphaLims(:,ii), this.musecLims(:,ii), ...
                                                                   alpha(ii));
    else
      ticks(ii) = interp1(this.alphaLims(end:-1:1,ii), this.musecLims(:,ii), ...
                                                                   alpha(ii));
    end
  end
  ticks = round(ticks);
  cmdstr = sprintf('#%d P%d ',[this.alphaIds ; ticks]);
  cmdstr = [cmdstr sprintf(' T%d \n', round(time*1000))];

  %fprintf(cmdstr); 			% Print actual command to STDOUT. To debug.
  fprintf(this.serialid, cmdstr);
  this.alpha = alpha;           % Keep track of last command.
  
  end

  %----------------------------- displayArm ----------------------------
  %
  %  Display the manipulator.  Joint angles should be in degrees since
  %  the are converted.  (Or remove the line giving the conversion and
  %  let it accept radians.  Up to you.)
  %
  %(
  function displayArm(this, alphadisp, linklen)

   if (nargin == 1)
    alphadisp = this.alpha;
  end

  vparms.home = 'straight-up';
%   alphadisp = diag([(pi/180)*ones([1 5]), 1])*alphadisp;	% Convert to rads.
  lynx6_display(alphadisp, linklen, vparms);

  end


  %)
  %----------------------------- forwardkin ----------------------------
  %
  %  Compute the forward kinematics of the manipulators.  If the
  %  manipulator was calibrated and the measurements taken so that it is
  %  possible to select either the middle of the grip region, or just
  %  the tip of the fingers, then totip would toggle between forward
  %  kinematics for one and the other.
  %
  %  Make sure that the SE3 class is in your path.
  %
  function g = forwardkin(this, alpha)
  

     mm2in   = 1/25.4;
  a1 = alpha(1);
  a2 = alpha(2);
  a3 = alpha(3);
  a4 = alpha(4);
  a5 = alpha(5);
 linklen = [110.6 120 120 130 20]*mm2in;	
  
  l0 = linklen(1);
  l1 = linklen(2);
  l2 = linklen(3);
  l3 = linklen(4);
  l4 = linklen(5);
  
  R_B_0 = eye(3);
  R_0_1 = [cos(a1) -sin(a1) 0; sin(a1) cos(a1) 0; 0 0 1];
  R_1_2 = [1 0 0; 0 cos(a2) -sin(a2); 0 sin(a2) cos(a2)];
  R_2_3 = [1 0 0; 0 cos(a3) -sin(a3); 0 sin(a3) cos(a3)];
  R_3_4pos = eye(3);
  R_4pos_4r = [1 0 0; 0 cos(a4) -sin(a4); 0 sin(a4) cos(a4)] * ...
      [cos(a5) -sin(a5) 0; sin(a5) cos(a5) 0; 0 0 1];
  R_4r_E = eye(3);

  g= SE3([0;0;l0], R_B_0) *...
      SE3([0;0;0], R_0_1) * ...
      SE3([0;0;0], R_1_2) * ...
      SE3([0; 0; l1], R_2_3) *...
      SE3([0;0;l2], R_3_4pos) * ...
      SE3([0;0;0], R_4pos_4r) * ...
      SE3([0;0;l3+l4], R_4r_E);

  overall_pos = getTranslation(g);
  end
  %----------------------------- inversekin ----------------------------
  %
  %  Compute the inverse kinematics of the manipulator.  Either to get
  %  the finger tips to the desired configuration or the middle of the
  %  grip region (second is default).  Also, solPick is an optional
  %  argument specifying which of the various solutions to choose from.
  %  At minimum, it should specify the upper or the lower solution.
  %  A second option would be to specify if it will be a turn around
  %  and reach backwards solution or a reach forwards.
  %
  %  Gripper width is ignored in these inverse kinematics, so only
  %  a 5x1 column vector is returned.
  %
  function alpha = inversekin(this, gdes)

  % specify link length
  mm2in   = 1/25.4;
  alpha = zeros(5,1);
 linklen = [110.6 120 120 130 20]*mm2in;	
  
  l0 = linklen(1);
  l1 = linklen(2);
  l2 = linklen(3);
  l3 = linklen(4);
  l4 = linklen(5);
  % g up to wrist, without gripper l4
  g_4r_E = [eye(3), [0;0;l4+l3]; 0,0,0,1];
  gWristPos = gdes * inv(g_4r_E);
  % position part
  xw = gWristPos(1, 4);
  yw = gWristPos(2, 4);
  zw = gWristPos(3, 4);
  % a3 has two chices
  rho_sq = xw^2 + yw^2 + (zw-l0)^2;
  a3_1 = acos((-rho_sq + (l1)^2 + l2^2)/(2*(l1)*l2));
  a3_2 = 2*pi - a3_1;
  a3_1 = a3_1 - pi;
  a3_2 = a3_2 - pi;
  % a2 has four choices
  w_1_1 = 2*l2 *sin(a3_1) + ...
      sqrt((2*l2*sin(a3_1))^2 - ...
      4*(l0-zw-l2*cos(a3_1)-l1)*(l0-zw+l2*cos(a3_1)+l1));
  w_1_2 = 2*l2 *sin(a3_1) - ...
      sqrt((2*l2*sin(a3_1))^2 - ...
      4*(l0-zw-l2*cos(a3_1)-l1)*(l0-zw+l2*cos(a3_1)+l1));
  w_2_1 = 2*l2 *sin(a3_2) + ...
      sqrt((2*l2*sin(a3_2))^2 - ...
      4*(l0-zw-l2*cos(a3_2)-l1)*(l0-zw+l2*cos(a3_2)+l1));
  w_2_2 = 2*l2 *sin(a3_2) - ...
      sqrt((2*l2*sin(a3_2))^2 - ...
      4*(l0-zw-l2*cos(a3_2)-l1)*(l0-zw+l2*cos(a3_2)+l1));
  w_1_1 = w_1_1 / (2*(l0 - zw - l2*cos(a3_1) - l1));
  w_1_2 = w_1_2 / (2*(l0 - zw - l2*cos(a3_1) - l1));
  w_2_1 = w_1_1 / (2*(l0 - zw - l2*cos(a3_2) - l1));
  w_2_2 = w_1_2 / (2*(l0 - zw - l2*cos(a3_2) - l1));
  a2_1_1 = 2*atan(w_1_1);
  a2_1_2 = 2*atan(w_1_2);
  a2_2_1 = 2*atan(w_2_1);
  a2_2_2 = 2*atan(w_2_2);
  a3a2set = [a3_1, a2_1_1; a3_1, a2_1_2; a3_2, a2_2_1; a3_2, a2_2_2];
  % a1 has 4 choice
  syms alpha1 alpha2 alpha3
  a1 = atan2(xw/(l2*sin(alpha2+alpha3)+l1*sin(alpha2)), -yw/(l2*sin(alpha2+alpha3)+l1*sin(alpha2)));
  a123 = zeros(3, 3);
  for i = 1:4
     a1_t = subs(a1, [alpha2, alpha3], [a3a2set(i, 2), a3a2set(i, 1)]);
     a123(i, :) = [a1_t, a3a2set(i, 2), a3a2set(i, 1)];
  end
  % pick one set of alphas
  wrist_pos = [l2*sin(alpha1)*sin(alpha2+alpha3)+l1*sin(alpha1)*sin(alpha2);...
      -l2*cos(alpha1)*sin(alpha2+alpha3) - l1*cos(alpha1)*sin(alpha2); ...
      l0+l2*cos(alpha2+alpha3) + l1*cos(alpha2)];
  wrist_pos_goal = [xw; yw; zw]
  a123_good = a123;
  for j = 1:4
      as = a123(j, :);
      wrist_pos_j = double(subs(wrist_pos, [alpha1 alpha2 alpha3], [as]));

      distance = sqrt((wrist_pos_j(1,1)-wrist_pos_goal(1,1))^2 +...
          (wrist_pos_j(2,1)-wrist_pos_goal(2,1))^2+...
          (wrist_pos_j(3,1)-wrist_pos_goal(3,1)^2));
      if (distance > .001)
          a123_good(j, :) = zeros(1,3);
      end
  end
  
  % orientation part
  a12345 =[];
  for k = 1: size(a123_good, 2)
      as = a123_good(k, :);
      if ~isequal(as, [0 0 0])
          a1 = as(1);
          a2 = as(2);
          a3 = as(3);
          gwrist_position = [eye(3), [0;0;l0]; 0,0,0,1] *...
              [cos(a1) -sin(a1) 0 0; sin(a1) cos(a1) 0 0; 0 0 1 0; 0 0 0 1] * ...
              [1 0 0 0; 0 cos(a2) -sin(a2) 0; 0 sin(a2) cos(a2) 0; 0 0 0 1] * ...
              [1 0 0 0; 0 cos(a3) -sin(a3) 0; 0 sin(a3) cos(a3) l1; 0 0 0 1] * ...
              [1 0 0 0; 0 1 0 0; 0 0 1 l2; 0 0 0 1];
          gh = gwrist_position \ (gdes * inv(g_4r_E));% SANITY CHECK TRANSLATION VECTOR SHOULD BE 0
          Rh = gh(1:3, 1:3);
          a5 = atan2(-Rh(1, 2), Rh(1,1)); % (-pi/2, pi/2)
          a4 = atan2(-Rh(2, 3), Rh(3, 3));
          a12345 = [a12345; [a1, a2, a3, a4, a5]];
      end
  end
  alpha = a12345';
  end
 
   %---------------------------- alphaSpline ----------------------------
  %
  %  
  %(
  function [alphas, tstep] = alphaSpline(this,alphai, alphaf, tspan)
      t = [0 tspan];
      tstep = .01;
      tt = 0:tstep:tspan;
      
      a1_i = alphai(1);
      a2_i = alphai(2);
      a3_i = alphai(3);
      a4_i = alphai(4);
      a5_i = alphai(5);
      
      a1_f = alphaf(1);
      a2_f = alphaf(2);
      a3_f = alphaf(3);
      a4_f = alphaf(4);
      a5_f = alphaf(5);
      
      a1s = spline(t, [0 a1_i a1_f 0], tt);
      a2s = spline(t, [0 a2_i a2_f 0], tt);
      a3s = spline(t, [0 a3_i a3_f 0], tt);
      a4s = spline(t, [0 a4_i a4_f 0], tt);
      a5s = spline(t, [0 a5_i a5_f 0], tt);
      
      plot(tt, a1s, tt, a2s, tt, a3s, tt, a4s, tt, a5s);
      alphas = [a1s; a2s; a3s; a4s; a5s];
  end
  
  %---------------------------- posJacobian ----------------------------
  %
  %  Computes the manipulator Jacobian for the position part only.
  %  The frame of reference is the world coordinate system (it is
  %  not done in body coordinates).
  %(
  function pJac = posJacobian(this, alpha)

  % Construction the Jacobian matrix.
  pJac = [];

  end

  %)
  %----------------------- genPositionTrajectory -----------------------
  %
  %  Given a desired position trajectory, generate the joint angles
  %  that will follow it.  Also assume that the initial joint angles
  %  are availalbe as an argument and that they map to the initial
  %  value of the position trajectory.
  %
  %  ptraj tspan should be valid over the duration of the actual
  %  tspan.  There are no checks here for that case, just assumed
  %  to be the case.
  %
%   function alphaTraj = genPositionTrajectory(this, traj, tspan)
% 
% 
% 
%   % Resolved rate code here.  Use posODE as the ode function.
% 
%  
% 
%     %------------------------- posODE ------------------------
%     %
%     %  this function can access the variables in 
%     %  genPositionTrajectory, so it is all good.
%     %  Access ptraj properly here.
%     %
%     function alphaDot = posODE(t, alpha)
% 
%     
%     
%     % Fill in the ODE here.
% 
%     % Make sure to convert angular rates from radians per second
%     % to degrees per second.  Use diag function to help you.
%     alphaDot = conversionHere;
% 
%     end
% 
%   end

  %)
  %--------------------------- genTrajectory ---------------------------
  %
  %  Given a desired position trajectory, generate the joint angles
  %  that will follow it.  Also assume that the initial joint angles
  %  are availalbe as an argument and that they map to the initial
  %  value of the position trajectory.
  %
  %  ptraj tspan should be valid over the duration of the actual
  %  tspan.  There are no checks here for that case, just assumed
  %  to be the case.
  %
  function jtraj = genPositionTrajectory(this, traj)


% get world velocity and time span
v_w = traj.pdot;
tspan = traj.tspan;
alphai = traj.pvec;
tf = tspan;

% for loop: 1) body velocity xyz 2) body jacobian 3)pinv
mm2in   = 1/25.4;
linklen = [110.6 120 120 130 20]*mm2in;
alpha_tt = alphai;
ts = [0];
alphas = [alpha_tt];
dt = 1;

for tt = 1: dt : tf
%     % 1) body velocity in xyz
%     gtt = double(subs(g, [l0 l1 l2 l3 l4 a1 a2 a3 a4 a5], [linklen(1:end) alpha_tt(1:end)]));
%     ci_b_hat = gtt\[eye(3), v_w; 0,0,0,0];
%     ci_b = [ci_b_hat(1:3, 4); 0; 0; 0];
%     % 2) body jacobian
%     Jb_tt = double(subs(Jb, [l0 l1 l2 l3 l4 a1 a2 a3 a4 a5], [linklen(1:end) alpha_tt(1:end)]));
%     % 3) pinv for alpha dot
%     alpha_tt_dot = pinv(Jb_tt) * ci_b;
    % 4) update alphas
    [jtraj.time, jtraj.alpha]  =ode45(@groupODE,[0 tspan],alphai);
  
end


%------------------------ groupODE -----------------------
%
%  this function can access the variables in
%  genPositionTrajectory, so it is all good.
%  Access ptraj properly here.
%
    function aDot = groupODE(t, alpha)
        
        a1 = alpha(1);
        a2 = alpha(2);
        a3 = alpha(3);
        a4 = alpha(4);
        a5 = alpha(5);
        a6 = alpha(6);
        l0 = linklen(1);
        l1 = linklen(2);
        l2 = linklen(3);
        l3 = linklen(4);
        l4 = linklen(5);
        g1 = SE3([0;0;l0],SE3.RotZ(a1));
        g2 = SE3([0;0;0], SE3.RotX(a2));
        g3 = SE3([0;0;l1], SE3.RotX(a3));
        g4 = SE3([0;0;l2],SE3.RotX(a4));
        g5 = SE3([0;0;l3],SE3.RotZ(a5));
        g6 = SE3([0;0;l4],eye(3));
        g= g1 *g2 * g3 * g4 * g5 * g6;
        Jb = BodyJacobian(alpha);
        aDot = zeros(6,1);
        vel = traj.pdot;
        speedbody = getM(g) \ (SE3.hat(vel'));
        speedbody = SE3.unhat(speedbody);
        aDot = pinv(Jb(1:3, :))*speedbody(1:3);
    end

    function Jb = BodyJacobian(alphas)
        a1 = alphas(1);
        a2 = alphas(2);
        a3 = alphas(3);
        a4 = alphas(4);
        a5 = alphas(5);
        a6 = alphas(6);
        l0 = linklen(1);
        l1 = linklen(2);
        l2 = linklen(3);
        l3 = linklen(4);
        l4 = linklen(5);
        
        g1 = SE3([0;0;l0],SE3.RotZ(a1));
        g2 = SE3([0;0;0], SE3.RotX(a2));
        g3 = SE3([0;0;l1], SE3.RotX(a3));
        g4 = SE3([0;0;l2],SE3.RotX(a4));
        g5 = SE3([0;0;l3],SE3.RotZ(a5));
        g6 = SE3([0;0;l4],eye(3));
        
        Jb1 = [0 0 0 0 0 1]';
        Jb2 = [0 0 0 1 0 0]';
        Jb3 = [0 0 0 1 0 0]';
        Jb4 = [0 0 0 1 0 0]';
        Jb5 = [0 0 0 0 0 1]';
        Jb6 = [0 0 0 0 0 0]';
        J = [0 -1 1; 1 0 -1; -1 1 0];
        
        gs = [g1 g2 g3 g4 g5 g6];
        Js = [Jb1 Jb2 Jb3 Jb4 Jb5 Jb6];
        Jb = [];
        for i = 2:6
            gi = SE3([0;0;0],eye(3));
            for j = i:6
                gi =  gi * gs(j);
            end
            Jk = Js(:, i - 1);
            Jihat = [0 -Jk(6) Jk(5) Jk(1); Jk(6) 0 -Jk(4) Jk(2); -Jk(5) Jk(4) 0 Jk(3); 0 0 0 0];
            Jbi = gi.inv().adjoint(Jk);
            Jb = [Jb, Jbi];
        end
        Jb = [Jb, [0;0;0;0;0;0]];
    end
end

  %--------------------- followJointTrajectory --------------------
  %
  %  Given a desired joint trajectory, command the joint angles
  %  that will follow it for the desired time span passed as a
  %  second argument.
  %
  %  jtraj tspan should be valid over the duration of the desired
  %  tspan.  There are no checks here for that case, just assumed
  %  to be the case.
  %
  function followJointTrajectory(this, traj, tspan)

  % Code to command trajectory to follow here.
  %
  % 1. Generate linear spacing in time across tspan.
  % 2. Interpolate the alpha values for these times (use interp1).
  % 3. Compute the time differences between times.
  % 4. Using for loop from 2nd element to last element (1st is initial
  %     condition which we are at, so can ignore it), ...
  %    a] send manipulator to new joint witth time duration slightly
  %       more than delta t (say by 5-10%).
  %    b] wait for actual delta t.
  %    c] go to the next joint angle.
  %
    tt = diff(traj.time);
    ai = [traj.alpha(1,:)];
    this.setArm(ai');
    hold on
    pause(tt(1));
    for n = 2:length(traj.time)
      an = traj.alpha(n,:);
      this.setArm(an');
      pause(tt(n-1));
    end
  
  

  end

  function displayJointTrajectory(this, trajectory, tspan)
  
  tt = diff(trajectory.time);
  ai = trajectory.alpha(1,:);
  lynx6_display(ai');
  hold on
  pause(tt(1));
  for n = 2:length(trajectory.time)
      an = trajectory.alpha(n, :);
      lynx6_display(an');
      pause(tt(n-1));
  end
  
  
  
  end
  %------------------------------ shutdown -----------------------------
  %
  %  Send to sleep then close the serial port.
  %
  function shutdown(this)
  
  this.gotoSleep();
  fclose(this.serialid);
  
  end

end

%
%
%============================ Helper Functions ===========================
%
%

methods(Static)

  %-------------------------- closeSerialPort --------------------------
  %
  %  Find an open port using instrfind and then close it.  If the serial
  %  port is not closed for some funny reason, then call this helper
  %  method, as per:
  %
  %  lynx6.closeSerialPorts();
  %
  %  Should work.  Not tested.
  %
  function closeSerialPorts

  ports = instrfind();
 
  for ii=1:length(ports)
    if ( strcmp(ports(ii).Status,'open') )
      fclose(ports(ii));
    end
  end

  end

end


end
%
%================================= lynx6 =================================
