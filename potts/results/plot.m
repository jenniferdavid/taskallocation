X = load ("5_5.txt");
nrows = 5;
ncols = 5;
ndim = 2*nrows +ncols;
rdim = nrows + ncols;
for c = 1:ndim
    for r = 1:ndim
        if (X(r,c) > 1000)
            X(r,c) = 0;
        end
    end
end
X;
histogram(X);