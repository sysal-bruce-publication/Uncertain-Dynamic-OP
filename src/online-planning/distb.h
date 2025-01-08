#ifndef __DISTB_H__
#define __DISTB_H__

#include <boost/math/distributions/students_t.hpp>
#include "utils.h"

class LST
{
public:
	LST(const double alpha_0, const double kappa_0, const double mu_0,
		const double sigma_sq_0) : _alpha_0(alpha_0), _kappa_0(kappa_0),
		_mu_0(mu_0), _sigma_sq_0(sigma_sq_0) {}
	~LST() {}

	void inference(const vector<double>& samples);

	double getXValueWithCDF(const double cdf_prob) const
	{
		double val = boost::math::quantile(*_lst, cdf_prob);
		return _loc + val * _scale;
	}

	void getConfidenceInterval(const double percent, double& lb, double& ub) const;

private:
	double _df = 0;
	double _loc = 0;
	double _scale = 0;
	double _cl = 0;
	std::unique_ptr<boost::math::students_t> _lst;
	const double _alpha_0 = 0;
	const double _kappa_0 = 0;
	const double _mu_0 = 0;
	const double _sigma_sq_0 = 0;

	void _initLST();
};

#endif // !__DISTB_H__

