#pragma once
#ifndef _BUTTON_H_
#define _BUTTON_H_

#include <SFML/Graphics.hpp>
#include <string>

//Colours
namespace Colour
{
	const sf::Color teal(15, 255, 177, 255);
	const sf::Color deepBlue(30, 87, 240, 225);
	const sf::Color fadeRed(255, 0, 0, 175);
	const sf::Color fadeGreen(0, 255, 0, 175);
	const sf::Color fadeBlue(0, 0, 255, 175);
	const sf::Color fadePink(255, 128, 200, 175);
	const sf::Color fadeOrange(255, 128, 0, 175);
}

using std::string;

class Button
{
	sf::IntRect Box;
	string Text;
	sf::Font& font;
public:
	Button(int x, int y, int w, int h,sf::Font& pFont, string title = "NA");
	Button(sf::Vector2i coord, sf::Vector2i size, sf::Font& pFont, string title);

	void Draw(sf::RenderWindow &window, float sclFac = 1);
	bool isButtonPressed(const sf::Mouse, const int, const int);
};

#endif