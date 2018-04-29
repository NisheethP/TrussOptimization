#include "Button.h"

Button::Button(int x, int y, int w, int h, sf::Font& pFont, string title):
	font(pFont)
{
	Box.left = x;
	Box.top = y;
	Box.height = h;
	Box.width = w;
	Text = title;
}

Button::Button(sf::Vector2i coord, sf::Vector2i size, sf::Font& pFont, string title):
	font(pFont)
{
	Box.left = coord.x;
	Box.top = coord.y;
	Box.height = coord.y;
	Box.width = coord.x;
	Text = title;
}

void Button::Draw(sf::RenderWindow &window, float sclFac)
{
	sf::RectangleShape rect;
	rect.setPosition(sf::Vector2f(Box.left, Box.top));
	rect.setSize(sf::Vector2f(Box.width, Box.height));
	const int size = 50;

	sf::Text title;
	title.setPosition(0, 0);

	sf::Vector2f pos(Box.left, Box.top);
	title.setFont(font);
	title.setString(Text);
	title.setCharacterSize(size);
	title.setFillColor(Colour::deepBlue);

	sf::FloatRect bound = title.getLocalBounds();
	float sclX = 1, sclY = 1;
	sclY = (Box.height) / bound.height * 0.95*sclFac;
	sclX = (Box.width) / bound.width * 0.95*sclFac;

	//pos.x -= bound.left/2;
	pos.y -= std::ceil(bound.top / 2);

	if (sclX < sclY)
	{
		title.setCharacterSize(static_cast<int>(size * sclX));
		//sclY = sclX;
		//		bound = title.getLocalBounds();
		//		float Dif = Box.height - bound.height;
		//		Dif /= 2;
		//		pos.y += Dif;
	}
	else if (sclX > sclY)
	{
		title.setCharacterSize(static_cast<int>(size * sclY));
		//sclX = sclY;
		//		bound = title.getLocalBounds();
		//		float Dif = Box.width - bound.width;
		//		Dif /= 2;
		//		pos.y += Dif;
	}

	title.setPosition(pos);
	window.draw(rect);
	window.draw(title);
}

bool Button::isButtonPressed(const sf::Mouse mouse, const int x, const int y)
{
	if (!mouse.isButtonPressed(sf::Mouse::Button::Left))
		return false;
	else
		return Box.contains(x, y);
}