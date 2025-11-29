function d = angdiff(a, b)
% Kompatibel angdiff:
%  angdiff(x)        -> wrap x til (-pi, pi]
%  angdiff(a, b)     -> vinkel(b - a) wrappet til (-pi, pi]


n = nargin;
if n == 1
    x = a;
    d = atan2(sin(x), cos(x));
elseif n == 2
    d = atan2(sin(b - a), cos(b - a));
else
    error('angdiff: wrong number of inputs');
end
end

