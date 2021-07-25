function [x,e,wVec,etaVec,gVec] = HarzfeldModel(a,beta,e_pref,sigma,w0,paradigm)

trlNum = length(paradigm);
x = zeros(1,trlNum);
e = zeros(1,trlNum);
basis_num = length(e_pref);
w = w0;

% model outputs:
% x: estimation of environment
% e: error
wVec = zeros(basis_num,trlNum); % weights of the error sensetivity units
etaVec = zeros(1,trlNum); % error sensetivity 
gVec = zeros(basis_num,trlNum); % encoding of error in neurons

wVec(:,1) = w0';
for i = 1:trlNum-1
    % calculating error
    e(i) = paradigm(i) - x(i); % error
    
    % calculating error sensetivity based on error
    g = zeros(1,basis_num);
    for j = 1:basis_num
        g(j) = exp(-(e(i) - e_pref(j))^2/(2*sigma^2));
    end
    gVec(:,i+1) = g';
    eta = w * g'; % error sensetivity
    etaVec(i+1) = eta;
    
    % calculating new prediction and updating weights
    x(i+1) = a*x(i) + eta*e(i);
    
    if (i == 1)
        w = w0;
    else
        w = w + beta*sign(e(i-1)*e(i)) * (g)/(g*g');
        wVec(:,i+1) = w';
    end    
end



