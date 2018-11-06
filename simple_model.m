clf;
clear variables;
global q;
global a;
q = [0, 0];
q = q*pi/180;
a = [4, 3];

update(0, 0, 0);

f = figure(2);
clf(f);
offset = 60;
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
    b = uicontrol('Parent',f,'Style','slider','Position',[81,120+60*size(q, 2)+offset*(i-1),419,23],...
                  'value', a(i), 'min',0, 'max',10);
    bl1 = uicontrol('Parent',f,'Style','text','Position',[50,120+60*size(q, 2)+offset*(i-1),23,23],...
                    'String','0','BackgroundColor',bgcolor);
    bl2 = uicontrol('Parent',f,'Style','text','Position',[500,120+60*size(q, 2)+offset*(i-1),54,23],...
                    'String','10','BackgroundColor',bgcolor);
    bl3 = uicontrol('Parent',f,'Style','text','Position',[240,84+60*size(q, 2)+offset*(i-1),100,23],...
                    'String',strcat('a', int2str(i)),'BackgroundColor',bgcolor);
    bgcolor = f.Color;

    b.Callback = @(es, ed) update(es.Value, -1, i); 
end
function update(value, update_q, update_a)
    global q;
    global a;
    if update_q > 0
        q(update_q) = value*pi/180; 
    end
    if update_a > 0
        a(update_a) = value;
    end
    X0 = [0 0 0 1]';
    X1 = transformation_matrix(q(1), a(1))*X0;
    X2 = transformation_matrix(q(1), a(1))*transformation_matrix(q(2), a(2))*X0;
    generate_plot([X0 X1 X2]);
end
