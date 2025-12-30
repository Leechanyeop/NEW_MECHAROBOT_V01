#ifndef IR_ARRAY_H
#define IR_ARRAY_H

class IR_Array{
private:
	int pins[5];
public:
	IR_Array(int p0, int p1, int p2, int p3, int p4);  // 5개 핀 전달
	void readValues(int values[5]);
};

#endif // 