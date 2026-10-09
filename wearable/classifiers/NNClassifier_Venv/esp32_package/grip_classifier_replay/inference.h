#pragma once
#include <math.h>
#include "model_data.h"

inline void extractFeatures(const float samples[NN_WINDOW][2], float feature[2]) {
  for (int c=0; c<2; ++c) {
    float mean=0.0f, sum=0.0f;
    if (NN_CENTER_RAW) {
      for (int t=0; t<NN_WINDOW; ++t) mean+=samples[t][c];
      mean/=NN_WINDOW;
    }
    for (int t=0; t<NN_WINDOW; ++t) {
      float v=samples[t][c]-mean;
      sum+=v*v;
    }
    feature[c]=sqrtf(sum/NN_WINDOW);
  }
}

inline int predictGrip(const float feature[2], float probability[NN_CLASSES]) {
  float x[NN_INPUTS], h[NN_HIDDEN];
  for (int i=0; i<NN_INPUTS; ++i) x[i]=(feature[i]-NN_MEAN[i])/NN_SCALE[i];
  for (int j=0; j<NN_HIDDEN; ++j) {
    float z=NN_B1[j];
    for (int i=0; i<NN_INPUTS; ++i) z+=x[i]*NN_W1[i][j];
    h[j]=fmaxf(0.0f,z);
  }
  for (int k=0; k<NN_CLASSES; ++k) {
    probability[k]=NN_B2[k];
    for (int j=0; j<NN_HIDDEN; ++j) probability[k]+=h[j]*NN_W2[j][k];
  }
  int best=0;
  for (int k=1; k<NN_CLASSES; ++k) if (probability[k]>probability[best]) best=k;
  const float maximum=probability[best];
  float total=0.0f;
  for (int k=0; k<NN_CLASSES; ++k) { probability[k]=expf(probability[k]-maximum); total+=probability[k]; }
  for (int k=0; k<NN_CLASSES; ++k) probability[k]/=total;
  return best;
}
