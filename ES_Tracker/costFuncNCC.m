function [crossCorrelation] = costFuncNCC(delay, ref)
    crossCorrelation = sum(sum(ref.*conj(delay)))/sqrt(sum(sum(abs(ref).^2))*sum(sum(abs(delay).^2)));