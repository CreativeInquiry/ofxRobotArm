#include "models.hpp"
#include <iostream>
#include <cstdlib>  // malloc
#include <cstdio>   // sprintf

// KukaPose methods
using namespace kuka;
KukaPose::KukaPose()
{
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->a = 0;
    this->b = 0;
    this->c = 0;
}

KukaPose::KukaPose(float x, float y, float z, float a, float b, float c)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->a = a;
    this->b = b;
    this->c = c;
}

float KukaPose::get_x()
{
    return this->x;
}

float KukaPose::get_y()
{
    return this->y;
}

float KukaPose::get_z()
{
    return this->z;
}

float KukaPose::get_a()
{
    return this->a;
}

float KukaPose::get_b()
{
    return this->b;
}

float KukaPose::get_c()
{
    return this->c;
}

// SET ----------------------------------------------------------

void KukaPose::set_x(float x)
{
    this->x = x;
}

void KukaPose::set_y(float y)
{
    this->y = y;
}

void KukaPose::set_z(float z)
{
    this->z = z;
}

void KukaPose::set_a(float a)
{
    this->a = a;
}

void KukaPose::set_b(float b)
{
    this->b = b;
}

void KukaPose::set_c(float c)
{
    this->c = c;
}

float *KukaPose::get_translation()
{
    float translation[3] = {this->x, this->y, this->z};
    return translation;
}

float *KukaPose::get_rotation()
{
    float rotation[3] = {this->a, this->b, this->c};
    return rotation;
}

void KukaPose::print()
{
    std::cout << "KukaPose: x[" << this->x << "], y[" << this->y << "], z[" << this->z
              << "], a[" << this->a << "], b[" << this->b << "], c[" << this->c << "]" << std::endl;
}

//=====================================================================================================
// AXES methods

Axes::Axes()
{
    this->a1 = 0;
    this->a2 = 0;
    this->a3 = 0;
    this->a4 = 0;
    this->a5 = 0;
    this->a6 = 0;
}

Axes::Axes(float a1, float a2, float a3, float a4, float a5, float a6)
{
    this->a1 = a1;
    this->a2 = a2;
    this->a3 = a3;
    this->a4 = a4;
    this->a5 = a5;
    this->a6 = a6;
}

// GETTERS --------------------------------------------

float Axes::get_a1()
{
    return this->a1;
}

float Axes::get_a2()
{
    return this->a2;
}

float Axes::get_a3()
{
    return this->a3;
}

float Axes::get_a4()
{
    return this->a4;
}

float Axes::get_a5()
{
    return this->a5;
}

float Axes::get_a6()
{
    return this->a6;
}

// SETTERS --------------------------------------------

void Axes::set_a1(float a1)
{
    this->a1 = a1;
}

void Axes::set_a2(float a2)
{
    this->a2 = a2;
}

void Axes::set_a3(float a3)
{
    this->a3 = a3;
}

void Axes::set_a4(float a4)
{
    this->a4 = a4;
}

void Axes::set_a5(float a5)
{
    this->a5 = a5;
}

void Axes::set_a6(float a6)
{
    this->a6 = a6;
}

void Axes::print()
{
    std::cout << "Axes: a1[" << this->a1 << "], a2[" << this->a2 << "], a3[" << this->a3
              << "], a4[" << this->a4 << "], a5[" << this->a5 << "], a6[" << this->a6 << "]" << std::endl;
}

//=====================================================================================================
// DIGITAL OUTPUTS methods

DigitalOutputs::DigitalOutputs()
{
    this->o1 = 0;
    this->o2 = 0;
    this->o3 = 0;
}

float DigitalOutputs::get_o1()
{
    return this->o1;
}

float DigitalOutputs::get_o2()
{
    return this->o2;
}

float DigitalOutputs::get_o3()
{
    return this->o3;
}

void DigitalOutputs::set_o1(float o1)
{
    this->o1 = o1;
}

void DigitalOutputs::set_o2(float o2)
{
    this->o2 = o2;
}

void DigitalOutputs::set_o3(float o3)
{
    this->o3 = o3;
}

void DigitalOutputs::print()
{
    std::cout << "Digital Outputs: o1[" << this->o1 << "], o2[" << this->o2 << "], o3[" << this->o3 << "]" << std::endl;
}

//=====================================================================================================
// DATA methods

Data::Data()
{
    this->r_sol = KukaPose();
    this->r_ist = KukaPose();
    this->ai_pos = Axes();
    this->as_pos = Axes();
    this->ei_pos = Axes();
    this->as_pos = Axes();
    this->ma_cur = Axes();
    this->me_cur = Axes();
    this->delay = 0;
    this->dig_out = DigitalOutputs();
    this->ftc = KukaPose();
    this->ipoc = (int) NULL;
}

Data::Data(KukaPose r_ist, KukaPose r_sol, Axes ai_pos, Axes as_pos,
     Axes ei_pos, Axes es_pos, Axes ma_cur, Axes me_cur, unsigned short int delay,
     DigitalOutputs dig_out, KukaPose ftc, unsigned int ipoc)
{
    this->r_ist = r_ist;
    this->r_sol = r_sol;
    this->ai_pos = ai_pos;
    this->as_pos = as_pos;
    this->ei_pos = ei_pos;
    this->as_pos = as_pos;
    this->ma_cur = ma_cur;
    this->me_cur = me_cur;
    this->delay = delay;
    this->dig_out = dig_out;
    this->ftc = ftc;
    this->ipoc = ipoc;
}

KukaPose Data::get_r_ist()
{
    return this->r_ist;
}

KukaPose Data::get_r_sol()
{
    return this->r_sol;
}

Axes Data::get_ai_pos()
{
    return this->ai_pos;
}

Axes Data::get_as_pos()
{
    return this->as_pos;
}

Axes Data::get_ei_pos()
{
    return this->ei_pos;
}

Axes Data::get_es_pos()
{
    return this->es_pos;
}

Axes Data::get_ma_cur()
{
    return this->ma_cur;
}

Axes Data::get_me_cur()
{
    return this->me_cur;
}

unsigned short int Data::get_delay()
{
    return this->delay;
}

DigitalOutputs Data::get_digital_outputs()
{
    return this->dig_out;
}

KukaPose Data::get_ftc()
{
    return this->ftc;
}

unsigned int Data::get_ipoc()
{
    return this->ipoc;
}

void Data::set_r_ist(KukaPose r_ist)
{
    this->r_ist = r_ist;
}

void Data::set_r_sol(KukaPose r_sol)
{
    this->r_sol = r_sol;
}

void Data::set_ai_pos(Axes ai_pos)
{
    this->ai_pos = ai_pos;
}

void Data::set_as_pos(Axes as_pos)
{
    this->as_pos = as_pos;
}

void Data::set_ei_pos(Axes ei_pos)
{
    this->ei_pos = ei_pos;
}

void Data::set_es_pos(Axes es_pos)
{
    this->es_pos = es_pos;
}

void Data::set_ma_cur(Axes ma_cur)
{
    this->ma_cur = ma_cur;
}

void Data::set_me_cur(Axes me_cur)
{
    this->me_cur = me_cur;
}

void Data::set_delay(unsigned short int delay)
{
    this->delay = delay;
}

void Data::set_digital_outputs(DigitalOutputs dig_out)
{
    this->dig_out = dig_out;
}

void Data::set_ftc(KukaPose ftc)
{
    this->ftc = ftc;
}

void Data::set_ipoc(unsigned int ipoc)
{
    this->ipoc = ipoc;
}

void Data::print()
{
    std::cout << "Data: \n" << std::endl;
    std::cout << "R_Ist: " << std::endl;
    this->r_ist.print();
    std::cout << "R_Sol: " << std::endl;
    this->r_sol.print();
    std::cout << "Ai_Pos: " << std::endl;
    this->ai_pos.print();
    std::cout << "As_Pos: " << std::endl;
    this->as_pos.print();
    std::cout << "Ei_Pos: " << std::endl;
    this->ei_pos.print();
    std::cout << "Es_Pos: " << std::endl;
    this->es_pos.print();
    std::cout << "Ma_Cur: " << std::endl;
    this->ma_cur.print();
    std::cout << "Me_Cur: " << std::endl;
    this->me_cur.print();
    std::cout << "Delay: " << this->delay << std::endl;
    this->dig_out.print();
    std::cout << "FTC: " << std::endl;
    this->ftc.print();
    std::cout << "IPOC: " << this->ipoc << std::endl;
}
