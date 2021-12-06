1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Energy Density Based Algorithm Proposed                            %
%                                                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Field Dimensions - x and y maximum (in meters)
xm = 100;
ym = 100;
%x and y Coordinates of the Sink
%sink.x =0.5 * xm;
%sink.y = ym + 50;
%sink.x=50;
%sink.y=175;
sink.x = -100;
sink.y = -100;
%sink.x=0.5*xm;
%sink.y=0.5*ym;

%Number of Nodes in the field
n = 100;
%Optimal Election Probability of a node to become cluster head
%p=0.05;
PacketLength =4000;
ctrPacketLength = 125;
%Energy Model (all values in Joules)
%Initial Energy
Eo = 0.25;
%Eelec=Etx=Erx
ETX=50*0.000000001;
ERX=50*0.000000001;
%Transmit Amplifier types
Efs=10*0.000000000001;
Emp=0.0013*0.000000000001;
%Data Aggregation Energy
EDA=5*0.000000000001;

INFINITY = 999999999999999;
%maximum number of rounds
rmax=3000;
%%%%%%%%%%%%%%%%%%%%%%%%% END OF PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%
%Computation of do

do=sqrt(Efs/Emp);

%Creation of the random Sensor Network
figure(1);
for i=1:1:n
    S(i).xd(1) = rand(1,1)*xm;%×ø±ê
    %XR(i)=S(i).xd;
    S(i).yd(1) = rand(1,1)*ym;
    %YR(i)=S(i).yd;
    %S(i).G = 0;
    %initially there are no cluster heads only nodes
    S(i).type(1) = 'N';
    S(i).E(1) = Eo;
    %S(i).ENERGY(1) = 0;
    % hold on;
    plot (S(i).xd,S(i).yd,'og')
    hold on;
end

S(n+1).xd(1) = sink.x;
S(n+1).yd(1) = sink.y;
%plot(S(n+1).xd, S(n+1).yd, 'x')
hold on;

%First Iteration
figure(1);

%counter for CHs
countCHs=0;
%counter for CHs per round
rcountCHs=0;
cluster=1;

countCHs;
rcountCHs=rcountCHs+countCHs;
flag_first_dead=0;

TOT_ENERGY_INIT(1).E = 0;
for i = 1:1:n
    if (S(i).E(1)>0)
        TOT_ENERGY_INIT(1).E = TOT_ENERGY_INIT(1).E + S(i).E(1);
    end
end

k = 4;  %number of clusters

for r = 0:1:rmax
    r
    hold off;

    %Number of dead nodes
    dead=0;

    %counter for bit transmitted to Bases Station and to Cluster Heads
    packets_TO_BS=0;
    packets_TO_CH=0;
    %counter for bit transmitted to Bases Station and to Cluster Heads per round
    PACKETS_TO_CH(r+1)=0;
    PACKETS_TO_BS(r+1)=0;

    figure(1);

    for i=1:1:n
        %checking if there is a dead node
        if (S(i).E(1)<=0)
            dead=dead+1;
            S(i).type(1) = 'D';
            plot(S(i).xd, S(i).yd, 'red .');
            hold on;
            %S(i).active_flag(1) = 0;    %added by me
            %S(i).type(1) = 'D';
        end

        if (S(i).E(1) > 0)
            S(i).type(1)='N';
            plot(S(i).xd, S(i).yd, 'og')
            hold on;
        end
    end
    %plot(S(n+1).xd, S(n+1).yd, 'x')
    if (dead == n)
        break;
    end

    STATISTICS(r+1).DEAD = dead;
    DEAD(r+1)=dead;

    %When the first node dies
    if (dead==1)
        if(flag_first_dead==0)
            first_dead=r;
            flag_first_dead=1;
        end
    end

    %first round ALL nodes are contesting to
    %become a CH.  all should start off as active

    for i=1:1:n
        S(i).active_flag(1) = 1;
    end



    countCHs = 0;
    cluster = 1;


    for c = 1:1:k

        num_nebr_nodes = 0;
        Etot_nebr = 0;
        for i=1:1:n
            S(i).R(1) = xm/sqrt(pi*k);
            for j=1:1:n
                if (i~=j && S(i).active_flag(1) ==1 && S(j).active_flag(1) ==1)
                    distance_to_node = sqrt((S(i).xd(1) - S(j).xd(1))^2 + (S(i).yd(1) - S(j).yd(1))^2);
                    if (distance_to_node <= S(i).R(1))
                        num_nebr_nodes = num_nebr_nodes + 1;
                        Etot_nebr = Etot_nebr + S(j).E(1);
                    end
                end
            end
            S(i).U(1) = (S(i).E(1))/(Etot_nebr/num_nebr_nodes);
        end

        Max_U = 0;
        for i=1:1:n
            if((S(i).U(1) > Max_U) && (S(i).active_flag(1) == 1))
                Max_U = S(i).U(1);
                Max_U_ind = i;
            end
        end

        S(Max_U_ind).active_flag(1) = 0;
        S(Max_U_ind).type(1) = 'C';

        %neighbors nodes change their status to inactive
        for j = 1:1:n
            if (Max_U_ind ~= j && S(j).active_flag(1)==1) % N (to check dead node or not. not sure if we should consider dead node)or maybe can include active flag for dead nodes and check
                distance_to_node = sqrt((S(Max_U_ind).xd(1) - S(j).xd(1))^2 + (S(Max_U_ind).yd(1) - S(j).yd(1))^2);
                if ((distance_to_node <= S(Max_U_ind).R(1))&&(S(j).active_flag(1)==1))
                    S(j).active_flag(1) = 0;
                    S(j).CH_id(1) = Max_U_ind;
                end
            end
        end

        countCHs = countCHs+1;

        C(cluster).xd(1) = S(Max_U_ind).xd(1);
        C(cluster).yd(1) = S(Max_U_ind).yd(1);
        C(cluster).E(1) = S(Max_U_ind).E(1);
        plot(S(Max_U_ind).xd, S(Max_U_ind).yd, 'k*');

        distance_to_sink = sqrt((S(Max_U_ind).xd(1)-(S(n+1).xd(1)))^2 + (S(Max_U_ind).yd(1)-(S(n+1).yd(1)))^2);

        C(cluster).distance_to_sink(1) = distance_to_sink;
        C(cluster).id(1) = Max_U_ind;

        cluster=cluster+1;

    end

    %Energy Dissipated by each CH
    C(c).Erel(1)=0;
    C(c).Erel_id(1)=0;
    for c= 1:1:k
        C(c).Erel(1) = inf;
        for j = 1:1:k
            E_to_CH = 0;
            E_to_CH_id = 0;
            if(c ~= j)  %notice no sqrt because we are approximating energy using distance formula
                E_to_CH = ((C(c).xd(1)-(C(j).xd(1)))^2 + (C(c).yd(1)-(C(j).yd(1)))^2) + (C(j).distance_to_sink(1))^2;
                E_to_CH_id = j;
                if(E_to_CH < C(c).Erel(1))
                    C(c).Erel(1) = E_to_CH;
                    C(c).Erel_id(1) = E_to_CH_id;
                end

            end

        end

        C(c).Edir(1) = (C(c).distance_to_sink(1))^2;
    end

    for a=1:1:k

        %Energy dissipated
        if (C(a).Erel(1) < C(a).Edir(1))
            E_id = C(a).Erel_id(1);
            distance = sqrt(((C(a).xd(1))-(C(E_id).xd(1)))^2 + ((C(a).yd(1))-(C(E_id).yd(1)))^2);

            if (distance>do)
                C(a).E(1) = C(a).E(1)- ( (ETX+EDA)*(PacketLength) + Emp*PacketLength*( distance*distance*distance*distance ));
            end
            if (distance<=do)
                C(a).E(1) = C(a).E(1) - ( (ETX+EDA)*(PacketLength)  + Efs*PacketLength*( distance * distance ));
            end
            S(C(a).id(1)).E(1) = C(a).E(1);
        end

        if (C(a).Erel(1) >= C(a).Edir(1))
            distance = C(a).distance_to_sink(1);
            if (distance>do)
                C(a).E(1) = C(a).E(1)- ( (ETX+EDA)*(PacketLength) + Emp*PacketLength*( distance*distance*distance*distance ));
            end
            if (distance<=do)
                C(a).E(1) = C(a).E(1) - ( (ETX+EDA)*(PacketLength)  + Efs*PacketLength*( distance * distance ));
            end
            S(C(a).id(1)).E(1) = C(a).E(1);
        end

        packets_TO_BS=packets_TO_BS+1;
        PACKETS_TO_BS(r+1)=packets_TO_BS;

    end

    STATISTICS(r+1).CLUSTERHEADS(1) = cluster-1;
    CLUSTERHS(r+1)= cluster-1;

    %IF any node is not taken care of then we assign it to closest CH
    for i = 1:1:n
        id = 0;
        d_CH = 0;
        S(i).distance_to_CH(1) = inf;
        if S(i).active_flag(1) == 1
            for j = 1:1:k
                d_CH = sqrt((C(j).xd(1)-(S(i).xd(1)))^2 + (C(j).yd(1)-(S(i).yd(1)))^2);
                if(d_CH < S(i).distance_to_CH(1))
                    S(i).distance_to_CH(1) = d_CH;
                    id = j;
                end
            end
            S(i).active_flag(1) = 0;
            S(i).CH_id(1) = id;
        end
    end

    %calculating for each sensor node its energy lost to transmit to its corresponding CH

    for i=1:1:n
        if((S(i).E(1) > 0) && (S(i).type(1) == 'N'))
            min_dis = sqrt((S(i).xd(1)-(S(S(i).CH_id(1)).xd(1)))^2 + (S(i).yd(1)-(S(S(i).CH_id(1)).yd(1)))^2);
            if(min_dis>=do)
                S(i).E(1) = S(i).E(1)-( ETX*(ctrPacketLength) + Emp*ctrPacketLength*( min_dis * min_dis * min_dis * min_dis));
            end
            if(min_dis <do)
                S(i).E(1) = S(i).E(1)-( ETX*(ctrPacketLength) + Efs*ctrPacketLength*( min_dis * min_dis));
            end

            if(min_dis>0)
                S(S(i).CH_id(1)).E(1) = S(S(i).CH_id(1)).E(1) - ( (ERX + EDA)*ctrPacketLength);
                PACKETS_TO_CH(r+1)=n-dead-cluster+1;
            end

        end
    end
    hold on;

    TOT_ENERGY(r+1).E = 0;
    for i=1:1:n
        if(S(i).E(1) > 0)
            TOT_ENERGY(r+1).E = TOT_ENERGY(r+1).E + S(i).E(1);
        end
    end

    ENERGY_CONSUMP(r+1).E = (2*(TOT_ENERGY_INIT(1).E) - TOT_ENERGY(r+1).E)/(TOT_ENERGY_INIT(1).E);


    countCHs;
    rcountCHs = rcountCHs + countCHs;

end
%hold off;

figure (2)

x=1:1:r;
y=1:1:r;

for i=1:1:r
    x(i)=i;
    y(i) = n - STATISTICS(i).DEAD;
end
plot(x,y,'r');
xlabel('Rounds');
ylabel('Number of Nodes Alive');
title('The Lifetime of the Sensor Nodes Network');
hold on;

figure (3)

d = 1:1:r;
e = 1:1:r;

for i=1:1:r
    d(i) = i;
    e(i) = ENERGY_CONSUMP(i).E;
end

plot(d,e,'b');

xlabel('Rounds');
ylabel('Energy consumption');
title('Energy Consumption per Round');
hold on;
