function [lambdap34z, lambdap56z, lambdap78z,lambdap34y, lambdap56y, ...
    lambdap78y,lambdap34x, lambdap56x, lambdap78x] = lambda_p()
%LAMBDA_P Summary of this function goes here
%   calculates lambdas p

    lambdap34z = [0 0 0 0 0 1];
    lambdap56z = [0 0 0 0 0 1];
    lambdap78z = [0 0 0 0 0 1];
    
    lambdap34y = [0,0,0,0,1,0];
    lambdap56y = [0,0,0,0,1,0];
    lambdap78y = [0,0,0,0,1,0];
    
    lambdap34x = [0,0,0,1,0,0];
    lambdap56x = [0,0,0,1,0,0];
    lambdap78x = [0,0,0,1,0,0];

end

