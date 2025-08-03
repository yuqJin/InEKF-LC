function y = laprnd(mu, beta, m, n)
% LAPRND Generate Laplace distributed random numbers
% y = laprnd(mu, beta, m, n) generates an m-by-n matrix of random numbers
% drawn from the Laplace distribution with parameters mu and beta.

% Generate uniform random numbers
u = rand(m, n) - 0.5;

% Transform uniform random numbers to Laplace distribution
y = mu - beta * sign(u) .* log(1 - 2 * abs(u));

end