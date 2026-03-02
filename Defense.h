#pragma once
#include <Arduino.h>
#include <Timer.h>

/**
 * ██████╗ ███████╗███████╗███████╗███╗   ██╗███████╗███████╗
 * ██╔══██╗██╔════╝██╔════╝██╔════╝████╗  ██║██╔════╝██╔════╝
 * ██║  ██║█████╗  █████╗  █████╗  ██╔██╗ ██║███████╗█████╗
 * ██║  ██║██╔══╝  ██╔══╝  ██╔══╝  ██║╚██╗██║╚════██║██╔══╝
 * ██████╔╝███████╗██║     ███████╗██║ ╚████║███████║███████╗
 * ╚═════╝ ╚══════╝╚═╝     ╚══════╝╚═╝  ╚═══╝╚══════╝╚══════╝
*/

/*
調整すること

ボール距離値(真ん中手前より値)

移動速度
*/

class Defense{
public:
    void setup(void);
    void defense_(int start_cord);
    void reset(void);
    int defense_hadling_timeget(void);
private:
    #define diff(a) a/abs(a)

    /// @warning inlineだからな！！！！

    /// @brief ダッシュ　内部割り込み
    /// @param  何もない
    void dash(bool tl);
    /// @brief 360正規化
    /// @param a 正規化する変数
    inline void norm360P(int &a){a%=360,a+=a<0?360:0;}
    /// @brief 360正規化
    /// @param a 正規化する変数
    /// @return 正規化された値
    inline int norm360(int a){a%=360;return a<0?a+360:a;}
    /// @brief 前方判定
    /// @param angle 判定する角度
    /// @return 前方ならtrue、そうでなければfalse
    inline bool isFront(int angle){norm360P(angle);return angle<=90||angle>=270;}
    /// @brief 符号が異なるか判定
    /// @param a 判定する変数１
    /// @param b 判定する変数２
    /// @return 符号が異なればtrue、同じならfalse
    // inline bool diff_signs(int a,int b){return (a>0&&b<0)||(a<0&&b>0);}
    inline bool diff_signs(float a,float b){
        // 0付近は符号判定できないので「異ならない」とする
        if(a == 0.0f || b == 0.0f) return false;
        return (a > 0.0f) != (b > 0.0f);
    }
    /// @brief 左右30度範囲内か判定
    /// @param angle 判定する角度
    /// @return 範囲内ならtrue、そうでなければfalse
    inline bool isInSide30(int angle){angle=norm360(angle);return ((unsigned)(angle-60)<=60)||((unsigned)(angle-240)<=60);}
    /// @brief 角度差取得
    /// @param a 角度１
    /// @param b 角度２
    /// @return 角度差
    inline static int getErr(int a,int b){int d=abs((a-b)%360);return d>180?360-d:d;}
    /// @brief リスケール
    /// @param min1 最小値1
    /// @param max1 最大値1
    /// @param min2 最小値1
    /// @param max2 最大値2
    /// @param value 値1での値(リスケール前の値)
    /// @return 2の範囲にリスケールされた値
    inline static float scaleRange(float min1,float max1,float min2,float max2,float value){return value<=min1?min2:value>=max1?max2:(value-min1)/(max1-min1)*(max2-min2)+min2;}
    /// @brief 角度からxy取得
    /// @param angle 角度
    /// @param x 角度からのx
    /// @param y 角度からのy
    /// @return 代入!
    inline void applyXY(int angle,float &x,float &y){float r=radians(angle);x=sin(r),y=cos(r);}
    /// @brief 4角のやつ
    /// @param deg 判定角度
    /// @return 誤差30で4角に入るか
    inline bool isDiagonalAngle(float deg){return(deg >= 30.0f  && deg <= 60.0f)||(deg >= 120.0f && deg <= 150.0f) ||(deg >= 210.0f && deg <= 240.0f) ||(deg >= 300.0f && deg <= 330.0f);}
    /// @brief トグルで戻るやつ　できれば使いたくないようにしたいけどまあ難しい

};