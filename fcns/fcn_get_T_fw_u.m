function Mf = fcn_get_T_fw_u(Rop)

e_3=[0;0;1];
Mf=[Rop*e_3, zeros(3,1),zeros(3,1),zeros(3,1); 
     zeros(3,1),Rop*e_3,zeros(3,1),zeros(3,1); 
     zeros(3,1),zeros(3,1),Rop*e_3,zeros(3,1);
     zeros(3,1),zeros(3,1),zeros(3,1),Rop*e_3];
end