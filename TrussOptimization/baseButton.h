#pragma once
#ifndef _BASE_BUTTON_H_
#define _BASE_BUTTON_H_
#include <SFML\Graphics.hpp>

/*
*Base class for the rendering of buttons.
*The handling of input is not done here. 
*Derive class for actual functionality.
*/
class baseButton
{
protected:
	sf::Rect<int> rect;
	sf::String str;

public:
	baseButton();
	virtual bool handleClick(int x, int y) = 0;
	void getText(sf::Text&, sf::Font&);
	sf::Vector2f getPos();

	~baseButton();
};

#endif