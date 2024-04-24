function qd_ive= conver_qd(qd,qs,qm)
D=0.08;
L_lx=0.15;
R=0.02;

A1= qm-qs+(-D-L_lx)/R;
A2= qs-qm+(-D-L_lx)/R;
if (A1<A2)
    A=A2;
else
    A=A1;
end
B1= qm-qs+(D-L_lx)/R;
B2= qs-qm+(D-L_lx)/R;
if (B1<B2)
    B=B2;
else
    B=B1;
end

qd_ive= A + qd*(B-A);
end
