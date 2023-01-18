//A ".cpp" file, to define the function of the class "LettreMorse.h" that will be used to convert letters and numbers to dots and dashes according to their morse code.

#include "LettreMorse.h"  // All the content in "LettreMorse.h" is copied in here
#include <string.h>  // Enables us to use the function "strcpy"

LettreMorse::LettreMorse(){}; 
    
char lettremodifiee[100]; // the array that will contain the translation of a character typed by the user, we chose a large number to avoid filling up the array quickly

// The corresponding morse code to each letter (example)
//A  .-  B -...  C -.-.  D -.. E . F ..-.
//G --. H ....  I ..  J .---  K -.- L .-..
//M --  N -.  O --- P .--.  Q --.-  R .-.
//S ... T - U ..- V ...-  W .-- X -..-
//Y -.--  Z --..

    
void LettreMorse::convertisseur(char lettre){ // the function used for translation, you could also use switch/case instead of if/else if in this function

strcpy(lettremodifiee,""); // This makes sure there is no undefined character in the string "lettremodifiee"
    
if (lettre=='A'|| lettre == 'a')
{
strcpy(lettremodifiee," . -"); // "strcy" function copies the quoted string to "lettremodifiee", one of many options available
Serial.print(lettremodifiee); // prints "lettremodifiee"
}
else if (lettre=='B'|| lettre == 'b')
{
strcpy(lettremodifiee," - . . . ");
Serial.print(lettremodifiee);
}
else if (lettre=='C'|| lettre == 'c')
{
strcpy(lettremodifiee," - . - . ");
Serial.print(lettremodifiee);
}
else if (lettre=='D'|| lettre == 'd')
{
strcpy(lettremodifiee," - . . ");
Serial.print(lettremodifiee);
}
else if (lettre=='E'|| lettre == 'e')
{
strcpy(lettremodifiee," . ");
Serial.print(lettremodifiee);
}
else if (lettre=='F'|| lettre == 'f')
{
strcpy(lettremodifiee," . . - . ");
Serial.print(lettremodifiee);
}
else if (lettre=='G'|| lettre == 'g')
{
strcpy(lettremodifiee," - - . ");
Serial.print(lettremodifiee);
}
else if (lettre=='H'|| lettre == 'h')
{
strcpy(lettremodifiee," . . . . ");
Serial.print(lettremodifiee);
}
else if (lettre=='I'|| lettre == 'i')
{
strcpy(lettremodifiee," . .");
Serial.print(lettremodifiee);
}
else if (lettre=='J'|| lettre == 'j')
{
strcpy(lettremodifiee," . - - -");
Serial.print(lettremodifiee);
}
else if (lettre=='K'|| lettre == 'k')
{
strcpy(lettremodifiee," - . -");
Serial.print(lettremodifiee);
}
else if (lettre=='L'|| lettre == 'l')
{
strcpy(lettremodifiee," . - . .");
Serial.print(lettremodifiee);
}
else if (lettre=='M'|| lettre == 'm')
{
strcpy(lettremodifiee," - -");
Serial.print(lettremodifiee);
}
else if (lettre=='N'|| lettre == 'n')
{
strcpy(lettremodifiee," - . ");
Serial.print(lettremodifiee);
}
else if (lettre=='O'|| lettre == 'o')
{
strcpy(lettremodifiee," - - - ");
Serial.print(lettremodifiee);
}
else if (lettre=='P'|| lettre == 'p')
{
strcpy(lettremodifiee," . - - . ");
Serial.print(lettremodifiee);
}
else if (lettre=='Q'|| lettre == 'q')
{
strcpy(lettremodifiee," - - . - ");
Serial.print(lettremodifiee);
}
else if (lettre=='R'|| lettre == 'r')
{
strcpy(lettremodifiee," . - .");
Serial.print(lettremodifiee);
}
else if (lettre=='S'|| lettre == 's')
{
strcpy(lettremodifiee," . . . ");
Serial.print(lettremodifiee);
}
else if (lettre=='T'|| lettre == 't')
{
strcpy(lettremodifiee," - ");
Serial.print(lettremodifiee);
}
else if (lettre=='U'|| lettre == 'u')
{
strcpy(lettremodifiee," . . - ");
Serial.print(lettremodifiee);
}
else if (lettre=='V'|| lettre == 'v')
{
strcpy(lettremodifiee," . . . - ");
Serial.print(lettremodifiee);
}
else if (lettre=='W'|| lettre == 'w')
{
strcpy(lettremodifiee," . - - ");
Serial.print(lettremodifiee);
}
else if (lettre=='X'|| lettre == 'x')
{
strcpy(lettremodifiee," - . . - ");
Serial.print(lettremodifiee);
}
else if (lettre=='Y'|| lettre == 'y')
{
strcpy(lettremodifiee," - . - - ");
Serial.print(lettremodifiee);
}
else if (lettre=='Z'|| lettre == 'z')
{
strcpy(lettremodifiee," - - . . ");
Serial.print(lettremodifiee);
}
else if (lettre=='0')
{
strcpy(lettremodifiee," - - - - - ");
Serial.print(lettremodifiee);
}
else if (lettre=='1')
{
strcpy(lettremodifiee," . - - - - ");
Serial.print(lettremodifiee);
}
else if (lettre == '2')
{
strcpy(lettremodifiee," . . - - - ");
Serial.print(lettremodifiee);
}
else if (lettre == '3')
{
strcpy(lettremodifiee," . . . - - ");
Serial.print(lettremodifiee);
}
else if (lettre == '4')
{
strcpy(lettremodifiee," . . . . - ");
Serial.print(lettremodifiee);
}
else if (lettre == '5')
{
strcpy(lettremodifiee," . . . . . ");
Serial.print(lettremodifiee);
}
else if (lettre == '6')
{
strcpy(lettremodifiee," - . . . . ");
Serial.print(lettremodifiee);
}
else if (lettre == '7')
{
strcpy(lettremodifiee," - - . . . ");
Serial.print(lettremodifiee);
}
else if (lettre == '8')
{
strcpy(lettremodifiee," - - - . ");
Serial.print(lettremodifiee);
}
else if (lettre == '9')
{
strcpy(lettremodifiee," - - - - . ");
Serial.print(lettremodifiee);
}
else if(lettre == ' ')
{
Serial.print(" ");
}
else{ // If no character of the above has been selected
Serial.println(" Unknown symbol! ");
}
strcpy(lettre,""); // This makes sure the character "lettre" is empty and will not show an undefined character, thus an unknown symbol
}
