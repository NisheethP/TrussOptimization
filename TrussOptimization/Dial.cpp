#include "Dial.h"

Dial::Dial(int* pVal, int pX, int pY, sf::String pDesc, sf::Font& pFont, int pWidth) :
	inc(pX + pWidth, pY, 20, 20, pFont, ">"),
	dec(pX, pY, 20, 20, pFont, "<"), 
	descriptor(pDesc),
	font(pFont)
{
	val = pVal;
	x = pX;
	y = pY;
}

void Dial::Draw(sf::RenderWindow &window, float sclFac)
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

void Dial::incVal(int x)
{
	*val = *val + x;
}

void Dial::decVal(int x)
{
	*val = *val - x;
}