[x,y,z]=cylinder([0,5,5,0],50);
z([1,2],:)=0;
z([3,4],:)=50;


figure(1)
for I
hm = mesh(x,y,z);
rotate(hm, [1 0 0], 60)
axis equal
