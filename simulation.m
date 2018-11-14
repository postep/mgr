clf;
clear variables;
global q;
global a;
global weight;
weight = 1;
q = zeros(1, 7);
q = q*pi/180;
a = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 3];

global Xm;
global Xn;
Xm = [0, 0, 0, 1]';
Xn = [0, 0, 0, 1]';


global f;
f = figure(2);
clf(f);
offset = 55;
for i=1:size(q, 2)
    bgcolor = f.Color;
    b = uicontrol('Parent',f,'Style','slider','Position',[81,54+offset*(i-1),419,23],...
                  'value', q(i), 'min',0, 'max',360);
    bl1 = uicontrol('Parent',f,'Style','text','Position',[50,54+offset*(i-1),23,23],...
                    'String','0','BackgroundColor',bgcolor);
    bl2 = uicontrol('Parent',f,'Style','text','Position',[500,54+offset*(i-1),54,23],...
                    'String','360','BackgroundColor',bgcolor);
    bl3 = uicontrol('Parent',f,'Style','text','Position',[240,25+offset*(i-1),100,23],...
                    'String',strcat('q', int2str(i)),'BackgroundColor',bgcolor);
    bgcolor = f.Color;

    b.Callback = @(es, ed) update(es.Value, i, -1); 
end

for i=1:size(a, 2)
    bgcolor = f.Color;
    b = uicontrol('Parent',f,'Style','slider','Position',[81,100+offset*size(q, 2)+offset*(i-1),419,23],...
                  'value', a(i), 'min',0, 'max',10);
    bl1 = uicontrol('Parent',f,'Style','text','Position',[50,100+offset*size(q, 2)+offset*(i-1),23,23],...
                    'String','0','BackgroundColor',bgcolor);
    bl2 = uicontrol('Parent',f,'Style','text','Position',[500,100+offset*size(q, 2)+offset*(i-1),54,23],...
                    'String','10','BackgroundColor',bgcolor);
    bl3 = uicontrol('Parent',f,'Style','text','Position',[240,84+offset*size(q, 2)+offset*(i-1),100,18],...
                    'String',strcat('a', int2str(i)),'BackgroundColor',bgcolor);
    bgcolor = f.Color;

    b.Callback = @(es, ed) update(es.Value, -1, i); 
end

i = 1;
global control_Xn;
control_Xn = uicontrol('Parent',f,'Style','text','Position',[50,50 + 60*(size(q, 2)+size(a, 2))+offset*(i-1)/2,500,23],...
                'String',['Punkt chwytaka: ', mat2str(Xn)],'BackgroundColor',bgcolor);
i = 2;
global control_Xm;
control_Xm = uicontrol('Parent',f,'Style','text','Position',[50,50 + 60*(size(q, 2)+size(a, 2))+offset*(i-1)/2,500,23],...
                'String',['Punkt masy: ', mat2str(Xm)],'BackgroundColor',bgcolor);
i = 3;
global control_M0;
control_M0 = uicontrol('Parent',f,'Style','text','Position',[50,50 + 60*(size(q, 2)+size(a, 2))+offset*(i-1)/2,500,23],...
                'String',['Moment sily: ', mat2str([0 0 0])],'BackgroundColor',bgcolor);
i = 4;
global control_Fg;
control_Fg = uicontrol('Parent',f,'Style','text','Position',[50,50 + 60*(size(q, 2)+size(a, 2))+offset*(i-1)/2,500,23],...
                'String',['Sila czujnika: ', mat2str([0 0 0])],'BackgroundColor',bgcolor);
i = 5;
global control_theta_n;
control_theta_n = uicontrol('Parent',f,'Style','text','Position',[50,50 + 60*(size(q, 2)+size(a, 2))+offset*(i-1)/2,500,23],...
                'String',['kat ostatniego stawu: ', mat2str([0])],'BackgroundColor',bgcolor);
update(0, 0, 0);


            
function update(value, update_q, update_a)
    global q;
    global a;
    global weight;
    global f;
    if update_q > 0
        q(update_q) = value*pi/180; 
    end
    if update_a > 0
        a(update_a) = value;
    end
    X0 = [0 0 0 1]';
    
    [ Fgn, M0, X, Xn, Xm, theta_n ] = model(X0, q, a, weight);
    
    global control_Xn;
    global control_Xm;
    global control_M0;
    global control_Fg;
    global control_theta_n;
    control_Xn.String = ['Punkt chwytaka: ', mat2str(Xn)];
    control_Xm.String = ['Punkt masy: ', mat2str(Xm)];
    control_M0.String = ['Momenty sily: ', mat2str(M0)];
    control_Fg.String = ['Sily: ', mat2str(Fgn)];
    control_theta_n.String = ['Kat ostatniego stawu: ', mat2str(theta_n)];
            
    generate_plot(X);
end
