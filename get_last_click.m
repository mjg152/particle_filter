function get_last_click(varargin)
   a = get(gca,'CurrentPoint');
   x = ceil(a(1,1));
   y = ceil(a(1,2));
   
   assignin('base','new_x_pos',x);
   assignin('base','new_y_pos',y);
end 
