#include "distb.h"

namespace CU = CommonUtils;

void LST::_initLST()
{
	_lst = std::make_unique<boost::math::students_t>(_df);
}

void LST::inference(const std::vector<double>& samples)
{
	size_t n = samples.size();
	double xs_mean = CU::vecElemsMean(samples);
	double s_sq_sum = 0;
	if (n > 1) {
		std::for_each(std::begin(samples), std::end(samples),
			[&s_sq_sum, xs_mean](const double elem) 
			{ 
				s_sq_sum += pow(elem - xs_mean, 2); 
			});
	}

	double alpha_n = _alpha_0 + n / 2.;
	_df = 2 * alpha_n;
	double kappa_n = _kappa_0 + n;
	_loc = (_kappa_0 * _mu_0 + n * xs_mean) / kappa_n;
	double beta_0 = (_alpha_0 - 1) * _sigma_sq_0;
	double weighted_avg_term = _kappa_0 * n * pow(xs_mean - _mu_0, 2) / kappa_n;
	double beta_n = beta_0 + 0.5 * (s_sq_sum + weighted_avg_term);
	_scale = sqrt(beta_n * (kappa_n + 1) / (alpha_n * kappa_n)) ;

	_initLST();
}

void LST::getConfidenceInterval(const double percent, double& lb, double& ub) const
{
	double lower_quantile = 0.5 * (1 - percent);
	double upper_quantile = 1 - lower_quantile;
	double t_lower_c = boost::math::quantile(*_lst, lower_quantile);
	double t_upper_c = boost::math::quantile(*_lst, upper_quantile);
	lb = _loc + t_lower_c * _scale;
	ub = _loc + t_upper_c * _scale;
}

