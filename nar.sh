#!/bin/bash

# Quick for Submodule Steps Script

OPTION="$1"

if [ "$OPTION" = "" ]; then
    printf "\nSpecify an action: submodule, forcebranch, status, build or exit\n"
    read -r OPTION
    printf "\nSelected option: $OPTION\n"
fi

if [ "$OPTION" = "exit" ]; then
    printf "\nExiting script...\n"
    exit 0
fi

if [ "$OPTION" = "build" ]; then
    # Step 5: Build the submodule project
    printf "\nBuilding the submodule project..."
    cd libs/3128-common || {
        printf "\nSubmodule directory not found. Ensure the submodule was added successfully."
        exit 1
    }
    ./gradlew build || {
        printf "\nFailed to build the submodule project. Resolve build issues and try again."
        exit 1
    }

    # Step 6: Build the main project
    printf "\nBuilding the main project..."
    cd ../.. || {
        printf "\nFailed to return to the main project directory."
        exit 1
    }
    ./gradlew build || {
        printf "\nFailed to build the main project. Resolve build issues and try again."
        exit 1
    }
fi

if [ "$OPTION" == "status" ]; then
    printf "\nChecking status of main project...\n"
    git status
    cd libs/3128-common || {
        printf "\nSubmodule directory not found. Ensure the submodule was added successfully."
        exit 1
    }
    printf "\nChecking status of submodule project...\n"
    git status
fi

if [ "$OPTION" == "init"]; then
    # Define paths
    SUBMODULE_DIR="libs/3128-common"
    SUBMODULE_FILE="$SUBMODULE_DIR/build.gradle"
    GIT_MODULES_FILE=".gitmodules"

    # Check if both the submodule directory and .gitmodules file exist
    if [[ -d "$SUBMODULE_DIR" && -f "$GIT_MODULES_FILE" ]]; then
        echo "Submodule directory and .gitmodules file found."
        
        # Check if build.gradle is present in the submodule directory
        if [[ ! -f "$SUBMODULE_FILE" ]]; then
            echo "build.gradle not found in $SUBMODULE_DIR. Updating submodule..."
            git submodule update --init
        else
            echo "build.gradle found. No need to update submodule."
        fi
    else
        echo "Either $SUBMODULE_DIR or $GIT_MODULES_FILE is missing. Exiting."
        exit 1
    fi
fi

if [ "$OPTION" = "submodule" ]; then
    # Step 1: Remove any existing directories or cache
    printf "\nRemoving existing directories or cache..."
    git rm -r --cached libs/3128-common || {
        printf "\nNo existing directories or cache present. Proceeding."
    }

    # Step 2: Clone the submodule from the repository
    printf "\nAdding the submodule from the repository..."
    git submodule add https://github.com/Team3128/3128-common.git libs/3128-common || {
        printf "\nFailed to add submodule. Check the repository URL and try again."
        printf "\nWould you like to force instead? (y/n)\n"
        read -r force
        if [ "$force" = "y" ]; then
            git submodule add -f https://github.com/Team3128/3128-common.git libs/3128-common
        fi
    }

    printf "\nWould you like to auto-update the settings.gradle and build.gradle? (y/n)\n"
    read -r update
    if [ "$update" = "y" ]; then
        # Step 3: Update settings.gradle
        printf "\nUpdating settings.gradle..."
        settings_file="settings.gradle"
        if [ -f "$settings_file" ]; then
            printf "\n\ninclude ':libs:3128-common'" >> "$settings_file"
            printf "\nproject(':libs:3128-common').projectDir = file('libs/3128-common')" >> "$settings_file"
        else
            printf "\n$settings_file not found. Please add the necessary lines manually."
            exit 1
        fi

        # Step 4: Update build.gradle
        printf "\nUpdating build.gradle..."
        build_file="build.gradle"
        if [ -f "$build_file" ]; then
            printf "\n\ndependencies {" >> "$build_file"
            printf "\n\timplementation project(':libs:3128-common')" >> "$build_file"
            printf "\n}" >> "$build_file"
        else
            printf "\n$build_file not found. Please add the necessary dependency manually."
            exit 1
        fi
    fi

    printf "\nWould you like to delete the jit pack vendor dependency? (y/n)\n"
    read -r remove
    if [ "$remove" = "y" ]; then
        printf "\nRemoving jit pack vendor dependency..."
        rm vendordeps/3128-common.json || {
            printf "\nNo existing directories or cache present. Proceeding."
        }
    fi

    # Step 5: Build the submodule project
    printf "\nBuilding the submodule project..."
    cd libs/3128-common || {
        printf "\nSubmodule directory not found. Ensure the submodule was added successfully."
        exit 1
    }
    ./gradlew build || {
        printf "\nFailed to build the submodule project. Resolve build issues and try again."
        exit 1
    }

    # Step 6: Build the main project
    printf "\nBuilding the main project..."
    cd ../.. || {
        printf "\nFailed to return to the main project directory."
        exit 1
    }
    ./gradlew build || {
        printf "\nFailed to build the main project. Resolve build issues and try again."
        exit 1
    }

    printf "\nScript executed successfully. Submodule setup complete!\n"
fi

if [ "$OPTION" = "forcebranch" ]; then
    BRANCH="$2"
    if [ -z "$BRANCH" ]; then
        printf "\nNo branch name provided. Exiting.\n"
        exit 1
    fi
    printf "\nSwitching main project to branch: $BRANCH\n"
    git checkout "$BRANCH" || {
        printf "\nFailed to checkout branch in main repo.\n"
        exit 1
    }

    if [ ! -d "libs/3128-common" ]; then
        printf "\nSubmodule directory not found. Attempting to initialize...\n"
        git submodule update --init
    fi

    if [ -d "libs/3128-common" ]; then
        cd libs/3128-common || exit 1
        printf "\nSwitching submodule to branch: $BRANCH\n"
        git checkout "$BRANCH" || {
            printf "\nFailed to checkout branch in submodule.\n"
            exit 1
        }
        cd ../..
    else
        printf "\nSubmodule directory still not found. Exiting.\n"
        exit 1
    fi
fi