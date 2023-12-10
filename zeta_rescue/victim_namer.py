"""
The sole purpose of this file is to generate random names for victims to use in
the description field. I just thought it would be fun to write.

Author: Jack Milman
"""
import random

name_list = ["Greg", "Steve", "Anne", "Lisa", "Monica", "John", "Jane", "Harold", "Ronald", "Rachel", "Eve", "Winston", "Gorthalax"]
occupation_list = ["Journalist", "Chef", "Police Officer", "CEO", "Middle Manager", "Sysadmin", "Engineer", "Doctor", "Priest", "Fireman", "Park Ranger", "Scientist", "Destroyer"]

def give_name():
    name_ind = random.randint(0, len(name_list))
    name = name_list.pop(name_ind)
    occupation_ind = random.randint(0, len(occupation_list))
    occupation = occupation_list.pop(occupation_ind)
    return name + " the " + occupation