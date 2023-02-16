float middle_of_3(float a, float b, float c) {
  float middle;
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  } else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    } else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

int pn(float a) {
  if (a > 0) return 1;
  if (a < 0) return -1;
  return 0;
}

float calibrate_k(float k, float p, float c) {
  if (pn(c) == pn(p)) {
    if (pn(c) == 1) {
      if (c > p) {
        k -= 0.05;
      } else {
        k += 0.01;
      }
    } else if (pn(c) == -1) {
      if (c <= p) {
        k -= 0.05;
      } else {
        k += 0.01;
      }
    }
  } else {
    k += 0.01;
  }
  if (k > 1.0) return 1.0;
  if (k < 0.2) return 0.2;
  return k;
}

class kalman {
public:
  float varVolt = 0.08;
  float varProcess = 1;
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
