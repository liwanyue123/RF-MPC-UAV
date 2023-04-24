function M = fcn_get_T_Mrb_u(p)

e3=[0 0 1]';
c_Mf=p.c_Mf;
 
M=c_Mf*[ e3, -e3 ,-e3, e3 ];

end