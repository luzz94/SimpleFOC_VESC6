clear;
close;
clc;

load("kneeangles_cut.mat")
gf_v = 10;
gf_a = 1;
ds_val = 1;

previous_i = 1;

%[delay,setpoint,range,P,I,D,Vel_limit]
rf_points_2 = [
    50,25,1,-1,-1,-1,25;
    25,-25,1,-1,-1,-1,25;
    0,-50,1,-1,-1,-1,10;
    200,-40,1,-1,-1,-1,6;
    0,100,1,-1,-1,-1,45;
    0,150,1,-1,-1,-1,25;
    50,100,1,-1,-1,-1,15;
    0,0,1,-1,-1,-1,30
    ];

ka_ds = downsample(kneeangles_cut,ds_val)
rf_points = zeros(size(ka_ds,1),7)

for l = 1:1:size(ka_ds,1)
    if l == 1
        rf_points(l,:) = [0,ka_ds(l)*gf_a,0.5,-1,-1,-1,((ka_ds(l+1)-ka_ds(l))*gf_v)] ;
    else
        rf_points(l,:) = [0,ka_ds(l)*gf_a,0.5,-1,-1,-1,((ka_ds(l)-ka_ds(l-1))*gf_v)] ;
    end
end


device = serialport("COM3",115200);

subplot(2,1,1);
h=plot(NaN,NaN,'r');
drawnow;
hold on

subplot(2,1,2)
d=plot(ka_ds,'b');
drawnow
hold on

tw = 10000;
drw_t = 3;
timeout = 4000;

pos = zeros(1,2000);
time = zeros(1,2000);

i = 1; % index for data acquisition
j = 1; % index for row of the point matrix
k = 1; % index for the step of the state space machine

while(1)


    if(k == 1 && rf_points(j,1) ~= -1)  % se il parametro è definito

        if(i >= previous_i + rf_points(j,k)) % aspetto e passo al parametro
            k=k+1;
        end

    else if (k == 1)
            k=k+1; % altrimenti passo subito al successivo
    end
    end

    if(k == 2)

        if(rf_points(j,4) ~= -1)
            writeline(device,"MP0");
        end
        if(rf_points(j,5) ~= -1)
            writeline(device,"MI0");
        end
        if(rf_points(j,6) ~= -1)
            writeline(device,"MD0");
        end
        if(rf_points(j,7) ~= -1)
            %writeline(device,"MLV" + string(rf_points(j,7)));
        end

        if(rf_points(j,7) ~= -1)

            writeline(device,"M" + string(rf_points(j,7)))

        end
        k=k+1;
    end

    if(k == 3)

        sgn = sign(rf_points(j,7));
        if (((sgn == 1) && (pos(i) > rf_points(j,2) - rf_points(j,3))) || ((sgn == -1) && (pos(i) < rf_points(j,2) + rf_points(j,3))))
            k=1;
            previous_i = i;

            j=j+1;

            if(j > size(rf_points,1))
                j=size(rf_points,1);
                k=4;
                %break
            end
        end

        if((i-previous_i) >= (timeout))
            break
        end

    end

    if (k == 4 )
        if((i-previous_i) >= (timeout))
            break
        end
    end

    line = readline(device);

    if (contains(line,"Data:"))
        i=i+1;
        data = extractAfter(line,"Data:");

        pat = asManyOfPattern(wildcardPattern(1,inf,"Except",whitespacePattern),1);
        data_array = extract(data,pat);
        pos(i) = str2double(data_array(1));
        time(i) = i;
    end

    if (rem(i,drw_t) == 0)
        if i<=tw
            set(h, 'YData', pos(1:i));
            set(h, 'XData', time(1:i));
        else
            set(h, 'YData', pos(i-tw:i));
            set(h, 'XData', time(i-tw:i));
            xlim([time(i-tw) time(i)]);
        end
        drawnow;
    end

    %toc
end