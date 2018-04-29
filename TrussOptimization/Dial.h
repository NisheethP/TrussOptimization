#pragma once
#ifndef _DIAL_H_
#define _DIAL_H_

#include "Button.h"

template <typename T = int>
class Dial
{
	T* val;
	Button inc;
	Button dec;
	int x;
	int y;

	int width;

	sf::String descriptor;
	sf::Font& font;
public:
	Dial(T* pVal, int x, int y, sf::String pDesc,sf::Font& pFont, int pWidth = 70);
	void Draw(sf::RenderWindow &window, float sclFac = 1);

	bool checkUp(const sf::Mouse mouse, const int x, const int y)
	{
		return inc.isButtonPressed(mouse, x, y);
	}
	bool checkDown(const sf::Mouse mouse, const int x, const int y)
	{
		return dec.isButtonPressed(mouse, x, y);
	}

	void incVal(T x = 1);
	void decVal(T x = 1);

	int getVal() { return *val; }
};

template<typename T>
Dial<T>::Dial(T* pVal, int pX, int pY, sf::String pDesc, sf::Font& pFont, int pWidth) :
	inc(pX + pWidth, pY, 20, 20, pFont, ">"),
	dec(pX, pY, 20, 20, pFont, "<"),
	descriptor(pDesc),
	font(pFont)
{
	val = pVal;
	x = pX;
	y = pY;
}

template<typename T>
void Dial<T>::Draw(sf::RenderWindow &window, float sclFac)
{
	sf::Text Text;
	Text.setString("");
	Text.setFont(font);
	Text.setPosition(x + 40, y);
	Text.setCharacterSize(20);
	Text.setString(std::to_string(*val));

	sf::Text description;
	description.setString(descriptor);
	description.setFont(font);
	description.setPosition(x, y - 30);
	description.setCharacterSize(20);

	dec.Draw(window, 0.8);
	inc.Draw(window, 0.8);
	window.draw(Text);
	window.draw(description);

}

template<typename T>
void Dial<T>::incVal(T x)
{
	*val = *val + x;
}

template<typename T>
void Dial<T>::decVal(T x)
{
	*val = *val - x;
}

#endif