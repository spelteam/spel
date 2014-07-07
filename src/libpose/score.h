#ifndef SCORE_H
#define SCORE_H

#include <string.h>

class LimbLabel
{

public:

    Score();
    Score(float sc, string name)

    Score & operator=(const Score &s);
    bool operator<(const Score &s) const;
    bool operator>(const Score &s) const;
    bool operator==(const Score &s) const;
    bool operator!=(const Score &s) const;

private:
    float score; //detection score
    string detName; //detector name


}
