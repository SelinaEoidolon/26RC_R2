#include "j3_clac.h"

#include "math.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/**
 * @brief 浮点钳位
 *
 * 例如理论上 cos(theta3) 应该落在 [-1,1]
 * 但浮点误差可能让它变成 1.0000001
 * 所以这里做一次保护。
 */
static float ik_clamp(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }

    if (x > max_val)
    {
        return max_val;
    }

    return x;
}

float IK_NormalizeAngle(float angle_rad)
{
    while (angle_rad >= (float)M_PI)
    {
        angle_rad -= 2.0f * (float)M_PI;
    }

    while (angle_rad < -(float)M_PI)
    {
        angle_rad += 2.0f * (float)M_PI;
    }

    return angle_rad;
}

float IK_DegToRad(float deg)
{
    return deg * (float)M_PI / 180.0f;
}

float IK_RadToDeg(float rad)
{
    return rad * 180.0f / (float)M_PI;
}

float IK_ModelToMotorCmd(float theta_model, const JointCalib_t *cal)
{
    float q_cmd;

    if (cal == 0)
    {
        return 0.0f;
    }

    /*
     * 公式：
     *   q_cmd = motor_ref + dir * (theta_model - model_ref)
     *
     * 含义：
     *   - motor_ref：上电记录到的实际电机角
     *   - model_ref：上电姿态在模型中的角
     *   - theta_model：逆解算出来的目标模型角
     */
    q_cmd = cal->motor_ref + ((float)cal->dir) * (theta_model - cal->model_ref);

    return IK_NormalizeAngle(q_cmd);
}

IK_Status_t Arm3R_IK_WithOffset(float x,
                                float y,
                                float z,
                                float d1,
                                float a2,
                                float a3,
                                float theta1_hint,
                                const JointCalib_t *j1,
                                const JointCalib_t *j2,
                                const JointCalib_t *j3,
                                IK_Result3R_WithOffset_t *result)
{
    float rho;       /* 水平投影距离 */
    float h;         /* 相对第二关节参考平面的高度差 */
    float D;         /* 余弦定理中间变量，理论上等于 cos(theta3) */
    float s3_abs;    /* |sin(theta3)| */
    float theta1;    /* joint1模型角 */
    float theta2;    /* joint2模型角 */
    float theta3;    /* joint3模型角 */
    float k1, k2;    /* 求 theta2 的中间变量 */
    uint8_t count = 0;

    /* -----------------------------
     * 1. 输入检查
     * ----------------------------- */
    if ((result == 0) || (j1 == 0) || (j2 == 0) || (j3 == 0))
    {
        return IK_ERR_NULL;
    }

    result->status = IK_ERR_PARAM;
    result->solution_count = 0;
    result->base_singular = 0;

    result->model_sol[0].valid = 0;
    result->model_sol[1].valid = 0;
    result->motor_sol[0].valid = 0;
    result->motor_sol[1].valid = 0;

    if ((a2 <= IK_EPS) || (a3 <= IK_EPS))
    {
        result->status = IK_ERR_PARAM;
        return IK_ERR_PARAM;
    }

    /* -----------------------------
     * 2. 求 theta1（模型角）
     *
     * theta1 = atan2(y, x)
     *
     * 若 x=y=0，则目标点落在基座 z 轴上，
     * 这时 theta1 不唯一，使用外部给定参考值。
     * ----------------------------- */
    if ((fabsf(x) < IK_EPS) && (fabsf(y) < IK_EPS))
    {
        theta1 = theta1_hint;
        result->base_singular = 1;
    }
    else
    {
        theta1 = atan2f(y, x);
    }

    theta1 = IK_NormalizeAngle(theta1);

    /* -----------------------------
     * 3. 转化为平面二连杆问题
     *
     * rho = sqrt(x^2 + y^2)
     * h   = z - d1
     *
     * 对应二维平面中的目标点 (rho, h)
     * ----------------------------- */
    rho = sqrtf(x * x + y * y);
    h   = z - d1;

    /* -----------------------------
     * 4. 用余弦定理解 theta3
     *
     * D = (rho^2 + h^2 - a2^2 - a3^2) / (2*a2*a3)
     *
     * 若 |D| > 1，目标点不可达
     * ----------------------------- */
    D = (rho * rho + h * h - a2 * a2 - a3 * a3) / (2.0f * a2 * a3);

    if ((D < -1.0f - 1e-5f) || (D > 1.0f + 1e-5f))
    {
        result->status = IK_ERR_UNREACHABLE;
        return IK_ERR_UNREACHABLE;
    }

    D = ik_clamp(D, -1.0f, 1.0f);

    /* |sin(theta3)| = sqrt(1 - D^2) */
    s3_abs = sqrtf(fmaxf(0.0f, 1.0f - D * D));

    /* =========================================================
     * 5. 第一组解
     *    theta3 = atan2(+sqrt(...), D)
     * ========================================================= */
    theta3 = atan2f(+s3_abs, D);

    /*
     * theta2 公式：
     *   theta2 = atan2(-rho, h) - atan2(a3*sin(theta3), a2 + a3*cos(theta3))
     */
    k1 = a2 + a3 * cosf(theta3);
    k2 = a3 * sinf(theta3);
    theta2 = atan2f(-rho, h) - atan2f(k2, k1);

    theta1 = IK_NormalizeAngle(theta1);
    theta2 = IK_NormalizeAngle(theta2);
    theta3 = IK_NormalizeAngle(theta3);

    result->model_sol[count].theta1 = theta1;
    result->model_sol[count].theta2 = theta2;
    result->model_sol[count].theta3 = theta3;
    result->model_sol[count].valid  = 1;

    result->motor_sol[count].q1_cmd = IK_ModelToMotorCmd(theta1, j1);
    result->motor_sol[count].q2_cmd = IK_ModelToMotorCmd(theta2, j2);
    result->motor_sol[count].q3_cmd = IK_ModelToMotorCmd(theta3, j3);
    result->motor_sol[count].valid  = 1;

    count++;

    /* =========================================================
     * 6. 第二组解
     *    theta3 = atan2(-sqrt(...), D)
     * =========================================================
     *
     * 若 s3_abs 很小，说明两组解几乎重合，不再重复保存。
     */
    if (s3_abs > IK_EPS)
    {
        theta3 = atan2f(-s3_abs, D);

        k1 = a2 + a3 * cosf(theta3);
        k2 = a3 * sinf(theta3);
        theta2 = atan2f(-rho, h) - atan2f(k2, k1);

        theta1 = IK_NormalizeAngle(theta1);
        theta2 = IK_NormalizeAngle(theta2);
        theta3 = IK_NormalizeAngle(theta3);

        result->model_sol[count].theta1 = theta1;
        result->model_sol[count].theta2 = theta2;
        result->model_sol[count].theta3 = theta3;
        result->model_sol[count].valid  = 1;

        result->motor_sol[count].q1_cmd = IK_ModelToMotorCmd(theta1, j1);
        result->motor_sol[count].q2_cmd = IK_ModelToMotorCmd(theta2, j2);
        result->motor_sol[count].q3_cmd = IK_ModelToMotorCmd(theta3, j3);
        result->motor_sol[count].valid  = 1;

        count++;
    }

    /* -----------------------------
     * 7. 填写输出结果
     * ----------------------------- */
    result->solution_count = count;
    result->status = IK_OK;

    return IK_OK;
}