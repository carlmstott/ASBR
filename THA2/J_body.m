function jac=J_body(robot)
r=robot;
B=[r.B.w; r.B.v]; %this gives us our twist vectors from our w's and v's

%alright now we're coding in the big leauges. Well the first thing we need
%to know is the number of joints our robot has. 
j=length(B); %number of joints
T=eye(4); %we are probably going to need a 4 sized i matrix
jac=zeros(6,j); %this is probably going to be computaonally faster
MexpBlist=MexpB(robot);

for i=j:-1:1 %this loop will increment backward, so I can caululate the
             %last column of my body jacobian first.

if i==j %as the final column of the body jacobain is just the final twist
        %vector, im going to attack it by itseld
jac(:,j)=B(:,j);
else

T=T*inv(MexpBlist(:,:,i+1)); %this is the matrix im going to be taking the adjoint of
                            %I am referencing W7L2 slide 7 here. This
                            %should increment such that I tack on the
                            %inverse of e^-1(B(i+1))(theta+1) on the end of
                            %my last iteration's T

jac(:,i)=Adj(T)*B(i);       %calculating my ith column of my body jacobian

end
end


end




