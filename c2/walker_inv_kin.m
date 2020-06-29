function th = walker_inv_kin(xy5, xy1)
%% find inverse kinematics, using an fsolve non-linear solving algorithm
global th0
fun = @(th_try)err_walker_inv_kin(xy5,xy1,th_try);

options = optimset('Display','off', 'Algorithm', 'levenberg-marquardt');
th = fsolve(fun, th0, options);

end

function err = err_walker_inv_kin(xy5, xy1, th)
%% solver function for inverse kinematics

xyfor = walker_fw_kin(th, xy1);
% error between desired xy and current tried xy
err = xy5 - xyfor(:,5);
err2 = 180 - (th(1) + th(2) + th(3))*180/pi;

err = [err; err2];

end