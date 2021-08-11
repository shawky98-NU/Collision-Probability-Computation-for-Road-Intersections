        import traci.constants
traci.start('sumo-gui -c ./Config.sumocfg --start');
SIM_STEPS = [1 2500];
beginTime = SIM_STEPS(1);
duration =  SIM_STEPS(2);
endTime =  SIM_STEPS(1) +  SIM_STEPS(2) - 1;
route1=[];
route2=[];
route3=[];
route4=[];
route5=[];
route6=[];
route7=[];
route8=[];
route9=[];
route10=[];
route11=[];
route12=[];
route13=[];
route14=[];
route15=[];
route16=[];
pro=[];
sigma=1;
speeeed=14;
for i=1:duration
    traci.simulation.step();
    
    Current_Vehicles=traci.vehicle.getIDList();
        for Position_Counter=1:length(Current_Vehicles)
            route=traci.vehicle.getRouteID(Current_Vehicles{Position_Counter});
            position=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
            x= position(1);
            y=position(2);
            if ( x > 990) && (x<1020) && ( y > 990) && (y<1020) 
                switch (route)
                    case 'route1'
                        route1=[route1 position];
                        x1=route1(1:2:end);
                        y1=route1(2:2:end);
                        [xx1,yy1,TT1]=UTurn(route1(1),997,speeeed);                        
                        plot(x1,y1,'*b'), hold on
                        drawnow;
                    case 'route2'
                        route2=[route2 position];
                        x2=route2(1:2:end);
                        y2=route2(2:2:end);
                        route4=[route4 traci.vehicle.getSpeed(Current_Vehicles{Position_Counter})];
                        [xx2,yy2,TT2]=strighty(route2(2),1020,speeeed);                        
                        plot(x2,y2,'*r'), hold on
                        drawnow;
                    case 'route3'
                        position3=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route3=[route3 position3];
                        route4=[route4 traci.vehicle.getAngle(Current_Vehicles{Position_Counter})];
                        route5=[route5 traci.vehicle.getSpeed(Current_Vehicles{Position_Counter})];
                        x3=route3(1:2:end);
                        y3=route3(2:2:end);
                        plot(x3,y3,'*g'), hold on
                        drawnow;                 
                        [xx yy T]=prediction(x3(1),1010,route5(1));
                    case 'route4'
                        position4=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route4=[route4 position4];
                        x4=route4(1:2:end);
                        y4=route4(2:2:end);
                        [x44,y44,t44]=TrunLeft1(x4(1),990,speeeed);
                        plot(x4,y4,'*c'), hold on
                        drawnow;       
                    case 'route5'
                        position5=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route5=[route5 position5];
                        x5=route5(1:2:end);
                        y5=route5(2:2:end);
                        plot(x5,y5,'b'), hold on
                        drawnow;
                    case 'route6'
                        position6=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route6=[route6 position6];
                        x6=route6(1:2:end);
                        y6=route6(2:2:end);
                        plot(x6,y6,'b'), hold on
                        drawnow;
                    case 'route7'
                        position7=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route7=[route7 position7];
                        route8=[route8 traci.vehicle.getAngle(Current_Vehicles{Position_Counter})];
                        route9=[route9 traci.vehicle.getSpeed(Current_Vehicles{Position_Counter})];                        
                        x7=route7(1:2:end);
                        y7=route7(2:2:end);
                        [x_99 y_99 T_99]=prediction(x7(1),1020,route9(1));
                        plot(x7,y7,'b'), hold on
                        drawnow;                    
                    case 'route8'
                        position8=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route8=[route8 position8];
                        x8=route8(1:2:end);
                        y8=route8(2:2:end);
                        plot(x8,y8,'b'), hold on
                        drawnow; 
                    case 'route9'
                        position9=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route9=[route9 position9];
                        x9=route9(1:2:end);
                        y9=route9(2:2:end);
                        plot(x9,y9,'b'), hold on
                        drawnow;
                    case 'route10'
                        position10=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route10=[route10 position10];
                        x10=route10(1:2:end);
                        y10=route10(2:2:end);
                        plot(x10,y10,'b'), hold on
                        drawnow;
                    case 'route11'
                        position11=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route11=[route11 position11];
                        x11=route11(1:2:end);
                        y11=route11(2:2:end);
                        plot(x11,y11,'b'), hold on
                        drawnow;                    
                    case 'route12'
                        position12=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route12=[route12 position12];
                        x12=route12(1:2:end);
                        y12=route12(2:2:end);
                        [xx12,yy12,TT12,ll]=stright(route12(1),990,1003,speeeed);                       
                        plot(x12,y12,'b'), hold on
                        drawnow;       
                    case 'route13'
                        position13=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route13=[route13 position13];
                        x13=route13(1:2:end);
                        y13=route13(2:2:end);
                        plot(x13,y13,'b'), hold on
                        drawnow;
                    case 'route14'
                        position14=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route14=[route14 position14];
                        x14=route14(1:2:end);
                        y14=route14(2:2:end);
                        plot(x14,y14,'b'), hold on
                        drawnow;
                    case 'route15'
                        position15=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route15=[route15 position15];
                        x15=route15(1:2:end);
                        y15=route15(2:2:end);
                        route16=[route16 16.6];                        
                        [xx15,yy15,TT15,ll]=stright(route15(1),1020,997,speeeed);
                        plot(x15,y15,'*b'), hold on
                        drawnow;              
                    case 'route16'
                        position16=traci.vehicle.getPosition(Current_Vehicles{Position_Counter});
                        route16=[route16 position16];
                        x16=route16(1:2:end);
                        y16=route16(2:2:end);
                        plot(x16,y16,'*b'), hold on
                        drawnow;
                end
            end
        end
end
% tt=(x15(end)-x15(1))/88;
% x15=x15(1):tt:x15(end);
% y15=997*(ones(1,89));
% 
% % tt=(y2(end)-y2(1))/99;
% % y2=y2(1):tt:y2(end);
% % x2=1003*(ones(1,100));
% % tt=(x12(end)-x12(1))/31;
% % x12=x12(1):tt:x12(end);
% % y12=1003*(ones(1,32));
% % % lane2x=[];
% % lane2y=[];
% % lane2t=[];
% % lane15x=[];
% % lane15y=[];
% % lane15t=[];
% ppp=[];
% % tt=(y2(end)-y2(1))/31;
% % y2=y2(1):tt:y2(end);
% % x2=1003*(ones(1,33));
% pro=[pro Point(x44,y44,t44,xx15,yy15,TT15)];
% [ppp,to]=probb(pro,x15,y15,x4,y4,speeeed);
% % ppp=ppp(1:2:end-1);
% % TTT=ppp(2:2:end);
% % for i=1:length(x15)
% %     [xxx,yyy,TT]=stright(x15(i),1010,16.6);
% %     lane15x=[lane15x;xxx];
% %     lane15y=[lane15y;yyy];
% %     lane15t=[lane15t;TT];
% % end
% % for i=1:length(x15)
% %     [xxxx,yyyy,TTT]=strighty(y2(i),1010,16.6);                        
% %     lane2x=[lane2x;xxxx];
% %     lane2y=[lane2y;yyyy];
% %     lane2t=[lane2t;TTT];
% % end
% 
% % for i =1:12
% %     pro=[pro TCC1(lane15x(i,:),lane15y(i,:),lane15t(i,:),lane2x(i,:),lane2y(i,:),lane2t(i,:))]
% % end
% % end
% % X=98.4:0.001:101.6;
% % x1=X-100;
% % yy=sqrt(4-power(x1,2))+88;
% % yy_1=-sqrt(4-power(x1,2))+88;
% 
% % for i=1:length(x)
% % xCenter = x(i);
% % yCenter = y(i);
% % theta = 0 : 0.01 : 2*pi;
% % radius = 3;
% % x1 = radius * cos(theta) + xCenter;
% % y1 = radius * sin(theta) + yCenter;
% % plot(x,y,x1, y1), hold on
% % drawnow


function [x,y,t]=TrunLeft1(xinital,xfinal,speed)
       D     =        []   ;
       t     =        [0]   ;
       a1    =        1002 ;
       b1    =        991.1;  
       c1    =        119.2 ;
       diff  =        (xfinal-xinital)/88;
       x     =        xinital:diff:xfinal; 
       y     =        a1*exp(-((x-b1)/c1).^2);
       for i=1:length(x)-1
           y_seq=(y(i)-y(i+1)).^2;
           x_seq=(x(i)-x(i+1)).^2;
           D=[D sqrt(x_seq+y_seq)];
       end
       for i=1:length(D)
           t=[t t(end)+D(i)/speed];
       end
end














function[xx,yy,T,L]=stright(x_inital,final_x,y,inital_speed)
        re=(final_x-x_inital)/88;
        xx=x_inital:re:final_x;
        yy=y*(ones(1,99));
        L=[];
        T=[0];
        for i=1:length(xx)-1
           xx_seq=(xx(i)-xx(i+1)).^2;
            L=[L sqrt(xx_seq)];
        end
        for i=1:length(L)
            T=[T T(end)+(L(i)/inital_speed)];
       end
end

function [xx,yy,T]=strighty(x_inital,final_x,inital_speed)
        re=(final_x-x_inital)/88;
        yy=x_inital:re:final_x;
        xx=1003*(ones(1,89));
        L=[];
        T=[0];
        for i=1:length(yy)-1
           xx_seq=(yy(i)-yy(i+1)).^2;
            L=[L sqrt(xx_seq)];
        end
        for i=1:length(L)
            T=[T T(end)+(L(i)/inital_speed)];
       end
end

function [xx,yy,T]=UTurn(x_inital,final_x,inital_speed)
       a1 =       992.9 ;
       b1 =       1000;
       c1 =       54.83;
       re=(final_x-x_inital)/31;
       xx=x_inital:re:final_x;
       yy=a1*exp(-((xx-b1)/c1).^2);
       L=[];
       T=[0];
       for i=1:length(yy)-1
           xx_seq=(xx(i)-xx(i+1)).^2;
           yy_seq=(yy(i)-yy(i+1)).^2;
           L=[L sqrt(xx_seq+yy_seq)];
       end
       for i=1:length(L)
            T=[T T(end)+(L(i)/inital_speed)];
       end
end

function [xx,yy,T]=prediction(x_inital,final_x,inital_speed)
       a1 =       997.1 ;
       b1 =     0.02168;
       c1 =       432.1;
       re=(final_x-x_inital)/31;
       xx=x_inital:re:final_x;
       yy=a1*sin(b1*xx+c1);
       L=[];
       T=[0];
       for i=1:length(yy)-1
           xx_seq=(xx(i)-xx(i+1)).^2;
           yy_seq=(yy(i)-yy(i+1)).^2;
           L=[L sqrt(xx_seq+yy_seq)];
       end
       for i=1:length(L)
            T=[T T(end)+(L(i)/inital_speed)];
       end
end
function [pro]=Point(x1,y1,T1,x2,y2,T2)
        D=[];
        TCC=[];
        Prob=[];
        pro=[];
        for i=1:length(x1)
            y_seq=(y1(i)-y2(i)).^2;
            x_seq=(x1(i)-x2(i)).^2;
            D=[D sqrt(x_seq+y_seq)];
        end
        for i=1:length(T2)
            TCC=[TCC T1(i)-T2(i)];
        end
        for i=1:length(D)
            if (D(i)<=3.5)
                index=i;
                disp(i);
                break;
            end
        end
        x=(x1(index)+x2(index))/2;
        y=(y1(index)+y2(index))/2;
        pro=[x y];
end

function [pppp,TCC2]=probb(pro,curren1_x,curren1_y,curren2_x,curren2_y,speed)
    pd2 = makedist('HalfNormal','mu',0,'sigma',1);    
    a=-0.09;
    speed1=speed;
    speed2=speed;
    pppp=[];
    TCC1=[];
    TCC2=[];
%     pro=[1005 1000];
    seqr1_x=(pro(1)-curren1_x).^2;
    seqr1_y=(pro(2)-curren1_y).^2;
    L1=sqrt(seqr1_x+seqr1_y);   
    for i =1:length(L1)
        speedo=speed1+a*i;
        TCC1=[TCC1 L1(i)/speedo];
    end
    seqr2_x=(pro(1)-curren2_x).^2;
    seqr2_y=(pro(2)-curren2_y).^2;
    L2=sqrt(seqr2_x+seqr2_y);
    for i =1:length(L2)
        speedo1=speed2+a*0.7;
        TCC2=[TCC2 L2(i)/speedo1];
    end

    TCC=abs(TCC1-TCC2);
    normal=pdf(pd2,TCC2);
    pppp=[pppp normal];
    
        
end