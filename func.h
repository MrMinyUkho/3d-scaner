float middle_of_3(float a, float b, float c) {
  float middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  }
  else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

class kalman {
  public:
    float varVolt = 1.5;
    float varProcess = 0.01;
    float Pc = 0.0;
    float G = 0.0;
    float P = 1.0;
    float Xp = 0.0;
    float Zp = 0.0;
    float Xe = 0.0;

    float f(float val) {
      Pc = P + varProcess;
      G = Pc / (Pc + varVolt);
      P = (1 - G) * Pc;
      Xp = Xe;
      Zp = Xp;
      Xe = G * (val - Zp) + Xp;
      return (Xe);
    }
};
