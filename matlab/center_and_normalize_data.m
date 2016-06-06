function [output_data,inputmean,inputscale] = center_and_normalize_data( dynamics_data )
%CENTER_AND_NORMALIZE_DATA Summary of this function goes here
%   Detailed explanation goes here
% output_data: N*D
dynamics_data=dynamics_data';

N = size(dynamics_data,1)/2;
M = size(dynamics_data,2);
output_data = dynamics_data;
inputmean=mean(output_data(1:N,:)')';
output_data(1:N,:) = output_data(1:N,:) - repmat(mean(output_data(1:N,:)')',1,M);

mx = max(max(output_data(1:N,:)'));
mn = min(min(output_data(1:N,:)'));
inputscale=max(abs(mx), abs(mn));
output_data(1:N,:) = output_data(1:N,:)./max(abs(mx), abs(mn));

output_data=output_data';
inputmean=inputmean';
end

