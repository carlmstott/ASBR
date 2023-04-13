
function ellipsoidPlotter(Jstorage, normCountA, normCountL)
hold OFF;

for i=1:length(Jstorage)
    ellipsoid_plot_linear(Jstorage(:,:,i),1);
    ellipsoid_plot_angular(Jstorage(:,:,i),1);
    subplot(3,2,3)
    [Liso(i), Aiso(i)]=J_isptrophy(Jstorage(:,:,i));
    plot(Liso);
    hold ON
    plot(Aiso);
    hold OFF
    legend("LiniarIso", "AnglularIso");

    subplot(3,2,4)
    [Lcondition(i),Acondition(i)]=J_condition(Jstorage(:,:,i));
    plot(Lcondition);
    hold ON
    plot(Acondition);
    hold OFF
    legend("Lcondition", "Acondition");

    subplot(3,2,5)
    plot(normCountA(1:i));
    legend("angular error");

    subplot(3,2,6)
     plot(normCountL(1:i));
    legend("liniar error");

    if i==1
        gif('kukaAnimationDLSEllipsoid.gif','Delaytime',1/4,'loopcount',15)
    else
        gif
    end


end