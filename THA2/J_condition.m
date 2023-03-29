

function [Lcondition, Acondition]=J_condition(Jacobian)

[Liso, Aiso]=J_isptrophy(Jacobian);

Lcondition=Liso^2;
Acondition = Aiso^2;

end