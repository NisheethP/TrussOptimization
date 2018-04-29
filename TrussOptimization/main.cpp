#include <SFML/Graphics.hpp>
#include <iostream>
#include "Truss.h"
#include <list>
#include <cmath>
#include "TrussFEM.h"
#include "Dial.h"

//Shows the current state of the programme
enum State
{
	Solving,						//Solving the problem
	SettingSimpleSupport,			//Setting Simple Supports on the GS
	SettingRollerSupport,			//Setting Roller Supports on the GS;
	SettingForceX,					//Setting the X direction force on the GS
	SettingForceY,					//Setting the Y direction force on the GS
	Initializing,					//Setting up the data for the GS
};

//Struct to hod the coordinates of a point.
struct Coord
{
	float x;
	float y;
	
	Coord(float px, float py) : x(px), y(py) {}
	sf::Vector2f sf_Vec() { return sf::Vector2f(x, y); }

	Coord operator-(Coord& pX)
	{
		return Coord(x - pX.x, y - pX.y);
	}
	Coord operator+(Coord& pX)
	{
		return Coord(x + pX.x, y + pX.y);
	}
};

const double PI = 3.14159265;

//Generate the GroundStructure Truss
void getGroundStructure(int sizex, int sizey, Truss* baseTruss);

//Convert between coordinates of nodes (on lattice points) to those on the screen (display coordinates)
Coord convert(Node& base, const Coord& FrameOrigin, int scale);

//Does one iteration of the optimality criterion approach
void optimizeMeanCompliance(TrussFEM* mesh, double maxVol, double Amin = 0.01, double Amax = 2);

int main()
{
	sf::Font font;
	sf::Mouse mouse;

	sf::Vector2f resolution;
	resolution.x = static_cast<float>(sf::VideoMode::getDesktopMode().width*0.9);
	resolution.y = static_cast<float>(sf::VideoMode::getDesktopMode().height*0.9);

	if (!font.loadFromFile("C:\\Windows\\Fonts\\FTLTLT.ttf"))
	{
		return -1;
	}

	sf::RenderWindow window(sf::VideoMode(resolution.x, resolution.y), "Truss Optimization");
	
	Coord FRAME_ORIGIN = { 50, static_cast<float>(resolution.y*0.95) };		//The coordinates of the first node. Origin at TOP-LEFT CORNER of screen. X rightwards; Y downwards;
	int sizeX = 6;			//Number of Nodes in X direction
	int sizeY = 6;			//Number of Nodes in Y direction
	int scaleFactor = (sizeX>=sizeY)?(-FRAME_ORIGIN.x + 0.6*resolution.x)/sizeX: (-FRAME_ORIGIN.y + 0.6*resolution.y) / sizeY;	//Scaling of the truss for rendering. The nodes are 1 unit apart. Spacing is scaled by this factor .
	double maxVolume = 1200;	//The volume constraint for the problem
	double Amin = 0.001;		//The lower bound on the area
	double Amax = 10;		//The upper bound on the area

	float baseLinkThickness = 1.f;
	float nodeRadius = 6.f;
	
	//Generating the ground structure
	Truss groundStructure;
	getGroundStructure(sizeX, sizeY, &groundStructure);
	
	//Vectors for the Shapes to be drawn
	std::vector<sf::CircleShape> shapeNodes;
	std::vector<sf::RectangleShape> shapeLinks;

	std::cout << "Number of links in GS: " << groundStructure.getLinks().size() << std::endl;		//DEBUG output for the number of links in the GroundStructure
	
	//DEBUG Code to verify the local stiffness matrix
	//Link tlink({ 0,0 }, { 1,1 });
	//std::cout << TrussFEM::generateLocalStiffness(tlink);

	groundStructure.initArea(3);
	

	TrussFEM mesh(&groundStructure);
	mesh.applySimpleSupport(0);
	mesh.applyRollerSupport(30);

	mesh.applyForceY(5, -15000);
	//mesh.applyForceX(5, -15000);
	int iteration = 0;

	const int SizeInitX = 0.75*resolution.x;
	const int SizeInitY = 0.15*resolution.y;
	const int SizeDelY = 60;

	//BUTTONS
	Dial sizeXDial(&sizeX, SizeInitX, SizeDelY * 0 + SizeInitY, "Nodes-X", font, 100);
	Dial sizeYDial(&sizeY, SizeInitX, SizeDelY * 1 + SizeInitY, "Nodes-Y", font, 100);

	State runState = Initializing;
	//Programme Loop. This is when the rendering is happening
	while (window.isOpen())
	{
		shapeNodes.clear();
		shapeLinks.clear();
	
		scaleFactor = (sizeX >= sizeY) ? (-FRAME_ORIGIN.x + 0.6*resolution.x) / sizeX : (FRAME_ORIGIN.y - 0.02*resolution.y) / sizeY;
		//std::cout << std::endl << scaleFactor;
		//Event handling for the code
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();
			if (event.type == sf::Event::MouseButtonPressed)
			{
				//Check for SizeX
				if (sizeXDial.checkUp(mouse, sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y))
				{
					sizeXDial.incVal();
					getGroundStructure(sizeX, sizeY, &groundStructure);
				}

				if (sizeXDial.checkDown(mouse, sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y) && sizeXDial.getVal() > 1)
				{
					sizeXDial.decVal();
					getGroundStructure(sizeX, sizeY, &groundStructure);
				}

				//Check for SizeY
				if (sizeYDial.checkUp(mouse, sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y))
				{
					sizeYDial.incVal();
					getGroundStructure(sizeX, sizeY, &groundStructure);
				}

				if (sizeYDial.checkDown(mouse, sf::Mouse::getPosition(window).x, sf::Mouse::getPosition(window).y) && sizeYDial.getVal() > 1)
				{
					sizeYDial.decVal();
					getGroundStructure(sizeX, sizeY, &groundStructure);
				}
			}
		}
		
		//Setting up the render for NODES on the screen
		for (int i = 0; i < groundStructure.getNodes().size(); i++)
		{
			Node curNode = groundStructure.getNodes().at(i);
			Coord tempCoord = convert(curNode, FRAME_ORIGIN, scaleFactor);
			sf::CircleShape tempCircle;
			tempCircle.setOrigin(sf::Vector2f(nodeRadius, nodeRadius));
			tempCircle.setRadius(nodeRadius);
			tempCircle.setPosition(tempCoord.sf_Vec());
			tempCircle.setFillColor(sf::Color(255, 255, 255));
			shapeNodes.push_back(tempCircle);
		}

		//Setting up the render for the LINKS on the screen
		for (int i = 0; i < groundStructure.getLinks().size(); i++)
		{
			Link curLink = groundStructure.getLinks().at(i);
			sf::RectangleShape tempRect;
			Coord tempCoord1 = convert(curLink.getNode()[0], FRAME_ORIGIN, scaleFactor);
			Coord tempCoord2 = convert(curLink.getNode()[1], FRAME_ORIGIN, scaleFactor);

			tempRect.setOrigin(0, static_cast<float>(baseLinkThickness*curLink.getArea() / 2));

			sf::Vector2f size = { 0,0 };
			size.x = static_cast<float>(curLink.getLength()*scaleFactor);
			size.y = static_cast<float>(baseLinkThickness*curLink.getArea());

			float angle = static_cast<float>(curLink.getSlope());
			angle *= static_cast<float>(-180 / PI);

			tempRect.setPosition(tempCoord1.sf_Vec());
			tempRect.setSize(size);
			tempRect.setRotation(angle);

			shapeLinks.push_back(tempRect);
		}

		for (int i = 0; i < groundStructure.getLinks().size(); i++)
		{
			double temp = Amax - Amin;
			double color = 1 / temp;
			color *= (shapeLinks[i].getSize().y - Amin);
			color *= 255;
			double k = 1;
			double red = static_cast<sf::Uint8>(color * k);
			double green = static_cast<sf::Uint8>((255 - color)*k);
			double blue = 50;
			shapeLinks[i].setFillColor(sf::Color(red, green, blue));
		}

		if (runState == Solving)
		{
			optimizeMeanCompliance(&mesh, maxVolume, Amin, Amax);
		}
		
		window.clear();

		if (runState == Initializing)
			sizeXDial.Draw(window);
		if (runState == Initializing)
			sizeYDial.Draw(window);

		for (int i = 0; i < shapeLinks.size(); i++)
			window.draw(shapeLinks[i]);

		//Drawing on the Window
		for (int i = 0; i < shapeNodes.size(); i++)
			window.draw(shapeNodes[i]);
			
		
		window.display();
		//printf("\nIter: %i \t Volume: %f \n ====================\n", iteration, mesh.getVol());
		iteration++;
		

		//VectorXd tempDisp = mesh.getDisplacement();
		//std::cout << std::endl << tempDisp << std::endl;
		//sf::sleep(sf::milliseconds(500));
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

//NUmerically approach the value of Solution
void optimizeMeanCompliance(TrussFEM* mesh, double maxVol, double Amin, double Amax)
{
	bool inInnerLoop = false;
	double beta = 0.5;
	double limVol = 0;

	mesh->assembleGlobal();
	mesh->solve();

	int size = mesh->getTruss().getLinks().size();
	double Lambda = 0;
	VectorXd Area(size);

	for (int i = 0; i < size; i++)
		Area(i) = mesh->getTruss().getLinks().at(i).getArea();

	for (int i = 0; i < size; i++)
	{
		double tempVar;

		VectorXd linkDisp(4);
		Link link = mesh->getTruss().getLinks().at(i);
		Matrix4d locStiff = TrussFEM::generateLocalStiffness(link);
		linkDisp = mesh->getLinkDisp(link);

		//std::cout << std::endl << linkDisp;
		//std::cout << std::endl << locStiff;

		tempVar = (linkDisp.transpose()*locStiff*linkDisp);
		tempVar /= maxVol;
		tempVar *= link.getLength()*link.getArea();
		Lambda += tempVar;
	}

	Lambda /= (maxVol - limVol);


	for (int i = 0; i < size; i++)
	{
		double tempVar;

		VectorXd linkDisp(4);
		Link link = mesh->getTruss().getLinks().at(i);

		inInnerLoop = false;

		//if (link.getArea() != Amin && link.getArea() != Amax)
		{
			Matrix4d locStiff = TrussFEM::generateLocalStiffness(link);
			linkDisp = mesh->getLinkDisp(link);

			tempVar = linkDisp.transpose()*locStiff*linkDisp;
			tempVar /= maxVol;

			double x = tempVar;
			x /= Lambda;
			std::cout << std::endl << i << ".  StrEng: " << tempVar;
			x = pow(x, beta);
			x *= Area(i);
			//std::cout << "\t Area: " << x;



			if (x >= Amin && x <= Amax)
				mesh->getTruss().updateArea(i, x);
			else if (x > Amax)
			{
				mesh->getTruss().updateArea(i, Amax);
				limVol += link.getArea()*link.getLength();
				inInnerLoop = true;
			}
			else if (x < Amin)
			{
				mesh->getTruss().updateArea(i, Amin);
				limVol += link.getArea()*link.getLength();
				inInnerLoop = true;
			}
		}
	}

	while (inInnerLoop)
	{
		double limVol = 0;

		mesh->assembleGlobal();
		mesh->solve();

		int size = mesh->getTruss().getLinks().size();
		double Lambda = 0;
		VectorXd Area(size);

		for (int i = 0; i < size; i++)
			Area(i) = mesh->getTruss().getLinks().at(i).getArea();

		for (int i = 0; i < size; i++)
		{
			double tempVar;

			VectorXd linkDisp(4);
			Link link = mesh->getTruss().getLinks().at(i);
			Matrix4d locStiff = TrussFEM::generateLocalStiffness(link);
			linkDisp = mesh->getLinkDisp(link);

			//std::cout << std::endl << linkDisp;
			//std::cout << std::endl << locStiff;

			tempVar = (linkDisp.transpose()*locStiff*linkDisp);
			tempVar /= maxVol;
			tempVar *= link.getLength()*link.getArea();
			Lambda += tempVar;
		}

		Lambda /= (maxVol-limVol);
		

		for (int i = 0; i < size; i++)
		{
			double tempVar;

			VectorXd linkDisp(4);
			Link link = mesh->getTruss().getLinks().at(i);

			inInnerLoop = false;

			if (link.getArea() != Amin && link.getArea() != Amax)
			{
				Matrix4d locStiff = TrussFEM::generateLocalStiffness(link);
				linkDisp = mesh->getLinkDisp(link);

				tempVar = linkDisp.transpose()*locStiff*linkDisp;
				tempVar /= maxVol;

				double x = tempVar;
				x /= Lambda;
				//std::cout << std::endl << i << ".  Lambda: " << tempVar;
				x = pow(x, beta);
				x *= Area(i);
				//std::cout << "\t Area: " << x;

				

				if (x >= Amin && x <= Amax)
					mesh->getTruss().updateArea(i, x);
				else if (x > Amax)
				{
					mesh->getTruss().updateArea(i, Amax);
					limVol += link.getArea()*link.getLength();
					inInnerLoop = true;
				}
				else if (x < Amin)
				{
					mesh->getTruss().updateArea(i, Amin);
					limVol += link.getArea()*link.getLength();
					inInnerLoop = true;
				}
			}
		}
	} 
	
	//while (inInnerLoop)
	{
		//double newVol = maxVol - limVol;
	}
}
