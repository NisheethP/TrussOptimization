#include "baseButton.h"

baseButton::baseButton()
{
}

void baseButton::getText(sf::Text& text, sf::Font& font)
{
	text.setString(str);
	text.setFont(font);
	text.setCharacterSize(30);
	sf::FloatRect bound = text.getLocalBounds();
	int offsetx = static_cast<int>((rect.width - bound.width) / 2);
	text.setPosition(rect.left + offsetx, rect.top + 8);
}

sf::Vector2f baseButton::getPos()
{
	return sf::Vector2f((float)rect.left, (float)rect.top);
}


baseButton::~baseButton()
{
}