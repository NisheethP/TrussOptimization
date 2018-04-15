#include <SFML/Graphics.hpp>
#include <iostream>
#include "Truss.h"
#include <list>
#include <cmath>
#include "TrussFEM.h"

//Shows the current state of the programme
enum State
{
	Solving,						//Solving the FEM, and optimizing the structure
	GroundStructure,				//Showing the GroundStructure (GS)
	SettingSimpleSupport,			//Setting Simple Supports on the GS
	SettingRollerSupport,			//Setting Roller Supports on the GS;
	SettingForceX,					//Setting the X direction force on the GS
	SettingForceY,					//Setting the Y direction force on the GS
	Initializing,					//Setting up the data for the GS
	Animating						//Animating the iteration to the optimal truss from GS;
};

struct Coord
{
	float x;
	float y;
	
	Coord(float px, float py) : x(px), y(py) {}
	sf::Vector2f sf_Vec() { return sf::Vector2f(x, y); }

	Coord operator-(Coord& pX)
	{
		return Coord(x-pX.x,y-pX.y);
	}
};

const double PI = 3.14159265;

//Generate the GroundStructure Truss
void getGroundStructure(int sizex, int sizey, Truss* baseTruss);

//Convert between coordinates of nodes (on lattice points) to those on the screen (display coordinates)
Coord convert(Node& base, const Coord& FrameOrigin, int scale);

//Standard insertion point for the programme. This has a console besides the windows being made. For DEBUG VERSION.
int main()
{
	sf::RenderWindow window(sf::VideoMode(1600, 900), "Truss Optimization");
	
	const Coord FRAME_ORIGIN = { 100,850 };		//The coordinates of the first node. Origin at TOP-LEFT CORNER of screen. X rightwards; Y downwards;
	int scaleFactor = 75;	//Scaling of the truss for rendering. The nodes are 1 unit apart. Spacing is scaled by this factor .
	int sizeX = 7;			//Number of Nodes in X direction
	int sizeY = 7;			//Number of Nodes in Y direction
	float baseLinkThickness = 1.f;
	float nodeRadius = 6.f;
	
	//Generating the ground structure
	Truss groundStructure;
	getGroundStructure(sizeX, sizeY, &groundStructure);
	
	//Vectors for the Shapes to be drawn
	std::vector<sf::CircleShape> shapeNodes;
	std::vector<sf::RectangleShape> shapeLinks;

	std::cout << groundStructure.getLinks().size() << std::endl;		//DEBUG output for the number of links in the GroundStructure
	
	//DEBUG Code to verify the local stiffness matrix
	//Link tlink({ 0,0 }, { 1,1 });
	//std::cout << TrussFEM::generateLocalStiffness(tlink);

	//Setting up the render for NODES on the screen
	for (int i = 0; i < groundStructure.getNodes().size(); i++)
	{
		Node curNode = groundStructure.getNodes().at(i);
		Coord tempCoord = convert(curNode, FRAME_ORIGIN,scaleFactor);
		sf::CircleShape tempCircle;
		tempCircle.setOrigin(sf::Vector2f(nodeRadius, nodeRadius));
		tempCircle.setRadius(nodeRadius);
		tempCircle.setPosition(tempCoord.sf_Vec());
		tempCircle.setFillColor(sf::Color(200, 100, 0));
		shapeNodes.push_back(tempCircle);
	}

	//Setting up the render for the LINKS on the screen
	for (int i = 0; i < groundStructure.getLinks().size(); i++)
	{
		Link curLink = groundStructure.getLinks().at(i);
		sf::RectangleShape tempRect;
		Coord tempCoord1 = convert(curLink.getNode()[0], FRAME_ORIGIN, scaleFactor);
		Coord tempCoord2 = convert(curLink.getNode()[1], FRAME_ORIGIN, scaleFactor);
		
		tempRect.setOrigin(0,baseLinkThickness/2);
		
		sf::Vector2f size = { 0,0 };
		size.x = static_cast<float>(curLink.getLength()*scaleFactor);
		size.y = static_cast<float>(baseLinkThickness);

		float angle = static_cast<float>(curLink.getSlope());
		angle *= -180 / PI;
		
		tempRect.setPosition(tempCoord1.sf_Vec());
		tempRect.setSize(size);
		tempRect.setRotation(angle);

		shapeLinks.push_back(tempRect);
	}
	
	//Programme Loop. This is when the rendering is happening
	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
		}


		window.clear();

		for (int i = 0; i < shapeLinks.size(); i++)
			window.draw(shapeLinks[i]);

		//Drawing on the Window
		for (int i = 0; i < shapeNodes.size(); i++)
			window.draw(shapeNodes[i]);
			
		
		window.display();
	}

	return 0;
}

//Generate the GroundStructure Truss
void getGroundStructure(int sizex, int sizey, Truss* baseTruss)
{
	std::vector<Node> nodes;
	for (int i = 0; i < sizex; i++)
	{
		for (int j = 0; j < sizey; j++)
			nodes.push_back({ i, j });
	}

	std::list<Link> links;
	std::vector<Link> linksVec;

	for (int i = 0; i < nodes.size()-1; i++)
	{
		for (int j = i+1; j < nodes.size(); j++)
		{
			links.push_back(Link(nodes[i], nodes[j]));
		}
	}
	std::list<Link> temp;
	
	for (std::list<Link>::iterator linkIter = links.begin(); linkIter != links.end(); ++linkIter)
	{
		temp.push_back(*linkIter);

		double Slope = linkIter->getSlope();
		double minLength = linkIter->getLength();

		for (std::list<Link>::iterator linkIter2 = linkIter; linkIter2 != links.end(); ++linkIter2)
		{
			
			if (linkIter->getNode()[0] == linkIter2->getNode()[0])
			{
				if ((linkIter != linkIter2) && Slope == linkIter2->getSlope())
				{
					temp.push_back(*linkIter2);
					if (linkIter2->getLength() < minLength)
						minLength = linkIter2->getLength();;
				}
			}
		}

		for (std::list<Link>::iterator linkIter2 = temp.begin(); linkIter2 != temp.end(); ++linkIter2)
		{
			if (linkIter->getNode()[0] == linkIter2->getNode()[0])
			{
				if (linkIter2->getLength() > minLength)
				{
					links.remove(*linkIter2);
				}
			}
		}
		
		linksVec.push_back(*temp.begin());
		temp.clear();
	}
	
	Truss tempTruss(nodes, linksVec);
	*baseTruss = tempTruss;
}

//Convert between coordinates of nodes (on lattice points) to those on the screen (display coordinates)
Coord convert(Node& base,const Coord& FrameOrigin, int scale)
{
	Coord newCoord = FrameOrigin;
	newCoord.x += scale * base.x;
	newCoord.y -= scale * base.y;

	return newCoord;
}