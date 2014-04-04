function [x_origin_circle,y_origin_circle,radius] = vview(B,plim,LMNplane,color)
  
% VVIEW - View the attainable virtual control set.
% 
%  1) vview(B,plim)
%
% Shows the attainable virtual control set considering actuator
% position constraints, given by { v : v = B*u, umin < u < umax }.
%
%  Inputs:
%  -------
% B      control effectiveness matrix (k x m)
% plim   control position limits [min max] (m x 2)
%
% The result is only graphically illustrated for k = 1, 2, or 3.
%
% See also: VVIEW_DEMO
  
% Model dimensions
  [k,m] = size(B);
  
  % ------------------------------------------------
  %  a) Find maximum attainable virtual control set 
  %     considering constraints.
  % ------------------------------------------------
  
  % Generate matrix to index corners of feasible control set.
  idx = zeros(2^m,m);
  M = 1:m;
  for i = 1:2^m;
    cbin = dec2bin(i-1,m); % '001'
    c = str2num(cbin')'; % [0 0 1]
    c = c(end:-1:1); % [1 0 0]
    idx(i,:) = 2*M - c;
  end

  % Generate corner points of the feasible control set.
  plimT = plim';
  U = plimT(idx)';

  % Compute the corresponding points in the virtual control space
  V = B*U;
  
  % Compute and visualize the convex hull of the set(s)
  
  switch k
     
   case 2
    [K,area1] = convhull(V(1,:),V(2,:));
    
    switch color 
        
        case 'b'
            color_plot='b';
        case 'r'
            color_plot='r';
        case 'g'
            color_plot='g';
        case 'm'
            color_plot='m';
        case 'c'
            color_plot='c';
        case 'k'
            color_plot='k';
        case 'y'
            color_plot='y';
    end
    
    
    %K
    %outline_points=[V(1,K)',V(2,K)']
    %all_points=[V(1,:)',V(2,:)']
    
    fill(V(1,K),V(2,K),color_plot); %red: [.9 0.0 0.0]; %std [.95 0.95 1]
    grid on;
    hold on;                          
    plot(V(1,K),V(2,K),color_plot)
    grid on;
    hold on;
  
    switch LMNplane
        
        case 'MN'
    axis equal;
    axis([-20 20 -30 30]);
    xlabel('M [Nm]')
    ylabel('N [Nm]')
    
         case 'LM'
    axis equal;
    axis([-30 30 -0.5 0.5]);
    xlabel('L [Nm]')
    ylabel('M [Nm]')
    
    x_origin_circle=0;
    y_origin_circle=0;
    length_L= length_L_fct(V(1,K),V(2,K),x_origin_circle,y_origin_circle)
    
         case 'LN'
    axis equal;
    axis([-30 30 -0.8 0.8]);
    xlabel('L [Nm]')
    ylabel('N [Nm]')
    
    x_origin_circle=0;
    y_origin_circle=0;
    length_L= length_L_fct(V(1,K),V(2,K),x_origin_circle,y_origin_circle)
    end
  
    %%
    %gives back the smallest distance between the center
    % (x_center,y_center) and the set of straight line created by the vertices
    % of a polynom (x_cooridant_vertex,y_cooridant_vertex)
    
    x_origin_circle=0;
    y_origin_circle=0;
    radius=circle_radius(V(1,K),V(2,K),x_origin_circle,y_origin_circle);
    
    %draw circle
    circle([x_origin_circle y_origin_circle],radius,3600,'k');

   case 3
    [K,vol1]    = convhulln(V');
    h = polyplot(K,V',1);
	set(h,'EdgeColor','b','FaceColor',[.95 .95 .99]);
    
    xlabel('L [Nm]')
    ylabel('M [Nm]')
    zlabel('N [Nm]')
    view(3);
    axis equal;
    axis vis3d;
    grid on;
   
  end

function [length_L]= length_L_fct(x_coordinat_vertex,y_coordinat_vertex,x_center,y_center)

% circle_radius - gives back the smallest distance between the center
% (x_center,y_center) and the set of straight line created by the vertices
% of a polynom (x_cooridant_vertex,y_cooridant_vertex)

Q=[x_coordinat_vertex' y_coordinat_vertex'];
P=[x_center,y_center];

distance_old=0;
 for i=(length(x_coordinat_vertex)+1)/2:(length(x_coordinat_vertex)-1);    %consider only right side of the plane
%for i=1:(length(x_coordinat_vertex)+1)/2;                                   %consider only left side of the plane

     distance_new = norm(det([Q(i+1,:)-Q(i,:);Q(i,:)-P]))/norm(Q(i+1,:)-Q(i,:));
     if distance_old>distance_new
         distance_max=distance_old;
     else
         distance_max=distance_new;
         distance_old=distance_new;
     end
end

length_L=distance_max;
end
 
function [min_radius_circle]= circle_radius(x_coordinat_vertex,y_coordinat_vertex,x_center,y_center)

% circle_radius - gives back the smallest distance between the center
% (x_center,y_center) and the set of straight line created by the vertices
% of a polynom (x_cooridant_vertex,y_cooridant_vertex)

Q=[x_coordinat_vertex' y_coordinat_vertex'];
P=[x_center,y_center];

distance_old=10000;

for i=1:length(x_coordinat_vertex)-1;

     distance_new = norm(det([Q(i+1,:)-Q(i,:);Q(i,:)-P]))/norm(Q(i+1,:)-Q(i,:));
     if distance_old<distance_new
         distance_min=distance_old;
     else
         distance_min=distance_new;
         distance_old=distance_new;
     end
end

min_radius_circle=distance_min;
end

function H=circle(center,radius,NOP,style)
%-----------------------------------------------------------

% H=CIRCLE(CENTER,RADIUS,NOP,STYLE)
% This routine draws a circle with center defined as
% a vector CENTER, radius as a scaler RADIS. NOP is 
% the number of points on the circle. As to STYLE,
% use it the same way as you use the rountine PLOT.
% Since the handle of the object is returned, you
% use routine SET to get the best result.
%
%   Usage Examples,
%
%   circle([1,3],3,1000,':'); 
%   circle([2,4],2,1000,'--');
%
%   Zhenhai Wang <zhe...@ieee.org>
%   Version 1.00
%   December, 2002
%-----------------------------------------------------------


if (nargin <3),
 error('Please see help for INPUT DATA.');
elseif (nargin==3)
    style='b-';
end;
THETA=linspace(0,2*pi,NOP);
RHO=ones(1,NOP)*radius;
[X,Y] = pol2cart(THETA,RHO);
X=X+center(1);
Y=Y+center(2);
H=plot(X,Y,style);
axis square;
end

function f = feasible(x,plim)
% x   m*n
% lb  m
% ub  m
  
  m = size(x,1);
  
  % Mean point
  x0 = mean(plim,2);
  
  % Make the mean point the origin
  x = x - x0*ones(1,size(x,2));
  lb = plim(:,1) - x0; % < 0
  ub = plim(:,2) - x0; % > 0
  
  % Check for feasibility
  tol = 1e-5;
  f = sum((diag(1./ub)*x <= 1+tol) & (diag(1./lb)*x <= 1+tol)) == m;
end

function h = polyplot(face,vert,merge)
  
  if merge 
    % Merge adjacent, parallel triangles to get fewer lines that
    % are not edges of the polyhedron.
    face4 = [];
    % Loop over all combinations of triangles
    k = 1;
    while k < size(face,1)
      l = k+1;
      while l <= size(face,1)
	iv = intersect(face(k,:),face(l,:)); % Intersecting vertices
	if length(iv) == 2 % Two common vertices
	  % Are the faces parallel?
	  niv = setxor(face(k,:),face(l,:)); % Non-intersecting vertices
	  % Vectors from first common vertex to remaining three vertices
	  A = [vert(iv(2),:)  - vert(iv(1),:);
	       vert(niv(1),:) - vert(iv(1),:);
	       vert(niv(2),:) - vert(iv(1),:)];
	  if abs(det(A))<100*eps
	    % Vectors lie in same plane -> create patch with four vertices
	    face4 = [face4 ; iv(1) niv(1) iv(2) niv(2)];
	    % ... and remove the two triangles
	    face = face([1:k-1 k+1:l-1 l+1:end],:);
	    k = k-1;
	    break
	  end	  
	end
	l = l+1;
      end % inner loop
      k = k+1;
    end % outer loop
    h = [patch('Faces',face,'Vertices',vert)
	 patch('Faces',face4,'Vertices',vert)];
  else
    % Just plot the polyhedron made up by triangles
    h = patch('Faces',face,'Vertices',vert);
  end
end
end

 