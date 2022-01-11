
%Left Wheel relative to CMM
x_1=
y_1=

%Right wheel relative to CMM
x_2=
y_2=

%Laser center relative to CMM
x_l=
y_1=

%Laser corners relative to CMM
x_3=
y_3=

x_4=
y_4=


%Base relative to CMM

c_x_b=(x_1+x_2)/2;
c_y_b=(y_1+y_2)/2;

theta=atan2(c_y_b,c_x_b);

b_T_c = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; -c_x_b -c_y_b 1];


%Laser relative to base

b_l=b_T_c*[x_l;y_l;1];

b_3=b_T_c*[x_3;y_3;1];

b_4=b_T_c*[x_4;y_4;1];

z_rot=atan2(b_4(2)-b_3(2),b_4(1)-b_3(1))



