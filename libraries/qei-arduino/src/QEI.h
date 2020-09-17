/*
 * QEI.h
 *
 * Created: 2017/03/19 11:38:23
 *  Author: Na-x4 <Na-x4@outlook.com>
 */


#ifndef QEI_H_
#define QEI_H_

class QEI {
    public:
        virtual ~QEI() {};
        virtual long read() = 0;
        virtual void write(long) = 0;
        virtual bool get_z() = 0;

        inline void reset() {
            write(0);
        }
};

#endif /* QEI_H_ */
