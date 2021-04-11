/*
    Author: Guilherme R. Moreira
    Local: University of São Paulo
    Creation: 2018, June
*/

#ifndef MODELS_HPP
#define MODELS_HPP
namespace kuka
{
  class KukaPose
  {
  private:
    float x;
    float y;
    float z;
    float a;
    float b;
    float c;

  public:
    KukaPose();
    KukaPose(float x, float y, float z, float a, float b, float c);
    //~Pose();

    float get_x();
    float get_y();
    float get_z();
    float get_a();
    float get_b();
    float get_c();

    void set_x(float x);
    void set_y(float y);
    void set_z(float z);
    void set_a(float a);
    void set_b(float b);
    void set_c(float c);

    float *get_translation();
    float *get_rotation();

    void print();
  };

  class Axes
  {
  private:
    float a1;
    float a2;
    float a3;
    float a4;
    float a5;
    float a6;

  public:
    Axes();
    Axes(float a1, float a2, float a3, float a4, float a5, float a6);
    //~Axes();

    float get_a1();
    float get_a2();
    float get_a3();
    float get_a4();
    float get_a5();
    float get_a6();

    void set_a1(float a1);
    void set_a2(float a2);
    void set_a3(float a3);
    void set_a4(float a4);
    void set_a5(float a5);
    void set_a6(float a6);

    void print();
  };

  class DigitalOutputs
  {
  private:
    bool o1;
    bool o2;
    bool o3;

  public:
    DigitalOutputs();

    float get_o1();
    float get_o2();
    float get_o3();

    void set_o1(float o1);
    void set_o2(float o2);
    void set_o3(float o3);

    void print();
  };

  class Data
  {
  private:
    KukaPose r_ist;
    KukaPose r_sol;
    Axes ai_pos;
    Axes as_pos;
    Axes ei_pos;
    Axes es_pos;
    Axes ma_cur;
    Axes me_cur;
    unsigned short int delay;
    DigitalOutputs dig_out;
    KukaPose ftc;
    unsigned int ipoc;

  public:
    Data();
    Data(KukaPose r_ist, KukaPose r_sol, Axes ai_pos, Axes as_pos,
         Axes ei_pos, Axes es_pos, Axes ma_cur, Axes me_cur, unsigned short int delay,
         DigitalOutputs dig_out, KukaPose ftc, unsigned int ipoc);

    KukaPose get_r_ist();
    KukaPose get_r_sol();
    Axes get_ai_pos();
    Axes get_as_pos();
    Axes get_ei_pos();
    Axes get_es_pos();
    Axes get_ma_cur();
    Axes get_me_cur();
    unsigned short int get_delay();
    DigitalOutputs get_digital_outputs();
    KukaPose get_ftc();
    unsigned int get_ipoc();

    void set_r_ist(KukaPose r_ist);
    void set_r_sol(KukaPose r_sol);
    void set_ai_pos(Axes ai_pos);
    void set_as_pos(Axes as_pos);
    void set_ei_pos(Axes ei_pos);
    void set_es_pos(Axes es_pos);
    void set_ma_cur(Axes ma_cur);
    void set_me_cur(Axes me_cur);
    void set_delay(unsigned short int delay);
    void set_digital_outputs(DigitalOutputs dig_out);
    void set_ftc(KukaPose ftc);
    void set_ipoc(unsigned int ipoc);

    void print();
  };
}
#endif //MODELS_HPP