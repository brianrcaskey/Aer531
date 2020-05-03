function [V,gamma,alpha,q,p,phi,beta,r,psi,N,E,alt] = stateparser(states)
    
    V = states(:,1);
    gamma = states(:,2);
    alpha = states(:,3);
    q = states(:,4);
    p = states(:,5);
    phi = states(:,6);
    beta = states(:,7);
    r = states(:,8);
    psi = states(:,9);
    N = states(:,10);
    E = states(:,11);
    alt = states(:,12);
    
end