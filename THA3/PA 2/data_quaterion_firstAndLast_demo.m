%solving the ax=xb equation using quartornian method, only using the first
%and last data point
%script written by Carl Stott on 4/23/22

clear all;

[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion();

%creating my M matrix (reference W12L1 slide 12)
for i=1:9:length(q_Robot_config)

Va=transpose(q_Robot_config(i,2:4)); %breaking up first quat, I assume the 
%firt entry in each row is the scaler component of the quat.
Sa=q_Robot_config(i,1);

Vb=transpose(q_camera_config(i,2:4));%breaking up second quart.

Sb=q_camera_config(i,1);

%expressing problem in Ax=b form (reference W12L1 slide 11), below is A in
%Ax=b eq.
A(:,:,i)=[Sa-Sb,-transpose(Va-Vb);(Va-Vb),(Sa-Sb)*eye(3) + skew(Va-Vb)];


%Now I need to generate my stack of Ra-I's W12L1S14
RaMinusI(:,:,i)=quart_to_rot(transpose(q_Robot_config(i,:)))-eye(3);


end

%below gives me M stack referenced in w12L1 slide 12
A=reshape(A,i*4,4); %


[U,S,V] = svd(A);

V=transpose(V); %smallest column of Vtranspose is the unit quartornian that
%represents rotation between robot end effector and camera, w12L61S3
QuartRx=V(:,4);
%verified that this outputs a unit quartornian

%turning QuartRx into a rotation matrix
Rx=quart_to_rot(QuartRx);


%Now I need to generate my stack ofRx*Pb-Pa's
for i=1:9:length(q_Robot_config)
Tb=transpose(t_camera_config(i,:));
Ta=transpose(t_Robot_config(i,:));

%the below generates the stack of B matricies we use in our least squares
%equation
stack(:,:,i)=Rx*Tb-Ta;
end

%reshaping my stack of Ra-I's to make a tall matrix
RaMinusI=reshape(RaMinusI,i*3,3);

%reshaping my stack of Rx*Tb-Ta's to make a tall matrix
stack=reshape(stack,i*3,1);

Tx=lsqr(RaMinusI,stack)

X=[Rx,Tx
   0,0,0,1]