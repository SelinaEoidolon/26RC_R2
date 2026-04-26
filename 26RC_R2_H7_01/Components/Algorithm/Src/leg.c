#include "leg.h"
#include "math.h"
#include "struct_typedef.h"
#define PI 3.1415926

//�ȵ���Ƕ� rad

leg lf_leg = {0,0};
leg rf_leg = {0,0};

float  lf_last_theta1 = LEGINIT_OFFSET;
float  lf_last_theta2 = LEGINIT_OFFSET;
float  rf_last_theta1 = LEGINIT_OFFSET;
float  rf_last_theta2 = LEGINIT_OFFSET;

float lb_leg = 0;
float rb_leg = 0;

//ǰ�ȴ�С�Ȳ���
float L1 = 94.5f;
float L2 = 112.5f;


//���Ȳ���


//�߶�cmd
int8_t LEG_Cmd = 0;
float bleg_theta = 0;
float fleg_high = 0;

//��ֹ��Ȧ
static float WrapToPi(float angle);
static float UnwrapNear(float angle, float ref);


//�������
float Q_rsqrt(float number) {
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = number * 0.5f;
    y = number;
    i = *(long*)&y;
    i = 0x5F3759DF - (i >> 1);
    y = *(float*)&i;
    y = y * (threehalfs - (x2 * y * y)); // һ��ţ�ٵ���
    return y;
}

//�����������
int InverseKinematics_Continuous(float x, float y,
                                 float last_theta1, float last_theta2,
                                 float *theta1, float *theta2)
{
    float L_sq = x * x + y * y;
    float L;
    float numerator, denominator;
    float cos_beta;
    float beta;
    float beta_limit;
    float phi;

    if (theta1 == 0 || theta2 == 0)
        return 0;

    /* ��ֹ���� */
    if (L_sq < 1e-6f)
        return 0;

    L = sqrtf(L_sq);

    /* �ɴ��� */
    if (L > (L1 + L2) || L < fabsf(L1 - L2))
        return 0;

    phi = atan2f(y, x);

    numerator   = L_sq + L1 * L1 - L2 * L2;
    denominator = 2.0f * L1 * L;

    cos_beta = numerator / denominator;

    /* ��ֹ acos ����� */
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;

    beta = acosf(cos_beta);

    beta_limit = 65.0f * 0.5f * PI / 180.0f;
    if (beta < beta_limit)
    {
        beta = beta_limit;
    }

    /* === ��ԭ���Ľ� === */
    float t1 = phi - beta;
    float t2 = PI - (phi + beta);

    /* === ?�ؼ��������� === */
    t1 = UnwrapNear(t1, last_theta1);
    t2 = UnwrapNear(t2, last_theta2);

    *theta1 = t1;
    *theta2 = t2;

    return 1;
}

static float WrapToPi(float angle)
{
    while (angle > PI)
        angle -= 2.0f * PI;

    while (angle <= -PI)
        angle += 2.0f * PI;

    return angle;
}

static float UnwrapNear(float angle, float ref)
{
    angle = WrapToPi(angle);

    while ((angle - ref) > PI)
        angle -= 2.0f * PI;

    while ((angle - ref) <= -PI)
        angle += 2.0f * PI;

    return angle;
}

void backLeg_Left(float h,float*theta)//�����˶��Ƕ�ȡ��ֵ
{
	//����λ�ã�����㣩
	
	//�߶ȽǶȽ���
	
	//�޷�����
}

void backLeg_Right(float h,float*theta)//�����˶��Ƕ�ȡ��ֵ
{
	//����λ�ã�����㣩
	
	//�߶ȽǶȽ���
	
	//�޷�����
}