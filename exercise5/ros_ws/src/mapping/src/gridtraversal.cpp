#include "gridtraversal.h"
#include <cmath>

static double tMax(double val, int step) {
    double t = std::floor(val + 1) - val;
    return (step == 1 || t == 1? t : 1.0 - t);
}

GridTraversal::GridTraversal(Vec2f p0, Vec2f p1) {
    mX0 = p0.x;
    mY0 = p0.y;
    mX1 = p1.x;
    mY1 = p1.y;

    double eps = 0.0001;
    mI = int(mX0 + eps);
    mJ = int(mY0 + eps);
    mI1 = int(mX1 + eps);
    mJ1 = int(mY1 + eps);

    mStepX = mStepY = 0;
    mTDeltaX = mTDeltaY = mTMaxX = mTMaxY = 0.0;

    if (mX1 != mX0 || mY1 != mY0) {
        mStepX = (mX1 >= mX0? 1 : -1);
        mStepY = (mY1 >= mY0? 1 : -1);

        if (mX1 == mX0) {
            mTDeltaY = 1.0;
            mTMaxY = tMax(mY0, mStepY);
        } else if (mY1 == mY0) {
            mTDeltaX = 1.0;
            mTMaxX = tMax(mX0, mStepX);
        } else {
            double m = (mY1 - mY0) / (mX1 - mX0);
            double m2 = m * m;
            mTDeltaX = std::sqrt(m2 + 1.0);
            mTDeltaY = std::sqrt(1.0 / m2 + 1.0);
            mTMaxX = tMax(mX0, mStepX) * mTDeltaX;
            mTMaxY = tMax(mY0, mStepY) * mTDeltaY;
        }
    }
}

Vec2i GridTraversal::get() {
    return Vec2i(mI, mJ);
}

bool GridTraversal::next() {
    bool hasNext = (((mStepX == 1? mI < mI1 : mI > mI1) && (mStepY == 1? mJ < mJ1 + 1 : mJ > mJ1 - 1))
                 || ((mStepY == 1? mJ < mJ1 : mJ > mJ1) && (mStepX == 1? mI < mI1 + 1 : mI > mI1 - 1)));

    if (!hasNext) {
        return false;
    }

    if ((mTMaxX < mTMaxY && mTDeltaX != 0.0) || mTDeltaY == 0.0) {
        mTMaxX += mTDeltaX;
        mI += mStepX;
    } else {
        mTMaxY += mTDeltaY;
        mJ += mStepY;
    }

    return true;
}
