#include "hchead.h"
#include <chrono>


bool nodeComparator(const tsNodeInfo& s1, const tsNodeInfo& s2)
{
	return s1.angle_q6_checkbit < s2.angle_q6_checkbit;
}

bool newComparator(const tsPointCloud& s1, const tsPointCloud& s2)
{
	return s1.dAngle > s2.dAngle;
}


HCHead::HCHead()
{

}

UINT64 HCHead::getCurrentTimestampNs()
{
	auto ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	return ts;
}

UINT64 HCHead::getCurrentTimestampUs()
{
    auto ts = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    return ts;
}

UINT64 HCHead::getCurrentTimestampMs()
{
	
	auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	return ts;
}


void HCHead::eraseBuff(std::vector<UCHAR>& lstG,int iLen)
{
	if (lstG.size() >= iLen)
		lstG.erase(lstG.begin(), lstG.begin() + iLen);
	else
		lstG.clear();

    {
         std::vector<UCHAR> tmp = lstG;
         lstG.swap(tmp);
    }
}


void HCHead::eraseRangeData(LstPointCloud& lstG,int iLen)
{
	if (lstG.size() >= iLen)
		lstG.erase(lstG.begin(), lstG.begin() + iLen);
	else
		lstG.clear();

    {
         LstPointCloud tmp = lstG;
         lstG.swap(tmp);
    }
}

double HCHead::getAngleFromXY(const double x, const double y)
{
	double a = atan2(y, x);
	double ret = a * 180 / PI_HC; //弧度转角度，方便调试
	if (ret > 360) {
		ret -= 360;
	}
	if (ret < 0) {
		ret += 360;
	}


	return 360 - ret;
}

double HCHead::getAngleFromAB(const double a, const double b)
{
	const double angle = atan(b / a) / PI_HC * 180;
	return angle;
}
double HCHead::getDistFromAB(const double a, const double b)
{
	const double dDist = sqrt(a*a + b * b);
	return dDist;
}

float HCHead::uint6_cov_float(UINT16 value)
{
	const Fp32 magic = { (254U - 15U) << 23 };
	const Fp32 was_infnan = { (127U + 16U) << 23 };
	Fp32 out;

	out.u = (value & 0x7FFFU) << 13;   /* exponent/mantissa bits */
	out.f *= magic.f;                  /* exponent adjust */
	if (out.f >= was_infnan.f)         /* make sure Inf/NaN survive */
	{
		out.u |= 255U << 23;
	}
	out.u |= (value & 0x8000U) << 16;  /* sign bit */

	return out.f;
}

UINT16 HCHead::float_cov_uint16(float value)
{

	const Fp32 f32infty = { 255U << 23 };
	const Fp32 f16infty = { 31U << 23 };
	const Fp32 magic = { 15U << 23 };
	const UINT32 sign_mask = 0x80000000U;
	const UINT32 round_mask = ~0xFFFU;

	Fp32 in;
	UINT16 out;

	in.f = value;

	uint32_t sign = in.u & sign_mask;
	in.u ^= sign;

	if (in.u >= f32infty.u) /* Inf or NaN (all exponent bits set) */
	{
		/* NaN->sNaN and Inf->Inf */
		out = (in.u > f32infty.u) ? 0x7FFFU : 0x7C00U;
	}
	else /* (De)normalized number or zero */
	{
		in.u &= round_mask;
		in.f *= magic.f;
		in.u -= round_mask;
		if (in.u > f16infty.u)
		{
			in.u = f16infty.u; /* Clamp to signed infinity if overflowed */
		}

		out = uint16_t(in.u >> 13); /* Take the bits! */
	}

	out = uint16_t(out | (sign >> 16));

	return out;
}


bool HCHead::lineFit(const std::vector<tsXY> &lstPoints, double &a, double &b, double &c)
{
	int size = lstPoints.size();
	if (size < 2)
	{
		a = 0;
		b = 0;
		c = 0;
		return false;
	}
	double x_mean = 0;
	double y_mean = 0;
	//for (int i = 0; i < size; i++)
	for (auto& sInfo : lstPoints)
	{
		x_mean += sInfo.fX;
		y_mean += sInfo.fY;
	}
	x_mean /= size;
	y_mean /= size; //至此，计算出了 x y 的均值

	double Dxx = 0, Dxy = 0, Dyy = 0;

	for (auto& sInfo : lstPoints)//for (int i = 0; i < size; i++)
	{
		Dxx += (sInfo.fX - x_mean) * (sInfo.fX - x_mean);
		Dxy += (sInfo.fX - x_mean) * (sInfo.fY - y_mean);
		Dyy += (sInfo.fY - y_mean) * (sInfo.fY - y_mean);
	}
	double lambda = ((Dxx + Dyy) - sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0;
	double den = sqrt(Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));
	a = Dxy / den;
	b = (lambda - Dxx) / den;
	c = -a * x_mean - b * y_mean;
	return true;
}
