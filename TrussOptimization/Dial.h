#pragma once
#ifndef _DIAL_H_
#define _DIAL_H_

#include "Button.h"

class Dial
{
	int* val;
	Button inc;
	Button dec;
	int x;
	int y;

	int width;

	sf::String descriptor;
	sf::Font& font;
public:
	Dial(int* pVal, int x, int y, sf::String pDesc,sf::Font& pFont, int pWidth = 70);
	void Draw(sf::RenderWindow &window, float sclFac = 1);

	bool checkUp(const sf::Mouse mouse, const int x, const int y)
	{
		return inc.isButtonPressed(mouse, x, y);
	}
	bool checkDown(const sf::Mouse mouse, const int x, const int y)
	{
		return dec.isButtonPressed(mouse, x, y);
	}

	void incVal(int x = 1);
	void decVal(int x = 1);

	int getVal() { return *val; }
};

#endif