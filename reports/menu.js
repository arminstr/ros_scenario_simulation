/**
 * Classes used for Menu
 * @author Armin Straller <armin.straller@hs-augsburg.de>
 */

export let menuApi = new MenuApi();

function MenuApi(){}

MenuApi .prototype .createMenu = function () {
    return new Menu();
}


MenuApi .prototype .createItem = function (text, callback) {
    return new MenuItem(text, callback);
}

MenuApi .prototype .createSeparator = function () {
    return new MenuSeparator();
}

function Menu(){
    this.bHidden = true;
    this.menuDiv = null;
    this.items = [];
    return this;
}

Menu .prototype .addItem = function(item) {
    if(item instanceof MenuItem || item instanceof MenuSeparator) {
        this.items.push(item);
        // this.render().appendChild(item.render());
        if(item instanceof MenuItem) {
            item.render().addEventListener('click', () => {
                item.callback(this);
            });
        }
    } else {
        return TypeError;
    }

}

Menu .prototype .addItems = function (...passedItems) {
    for (let i in passedItems) {
        if(passedItems[i] instanceof MenuItem || passedItems[i] instanceof MenuSeparator) {
            this.items.push(passedItems[i]);
            // this.render().appendChild(passedItems[i].render());
            if(passedItems[i] instanceof MenuItem) {
                passedItems[i].render().addEventListener('click', () => {
                    passedItems[i].callback(this);
                });
            }
        } else {
            return TypeError;
        }
    }
}

Menu .prototype .addItemAt = function (item, index) {
    if(item instanceof MenuItem || item instanceof MenuSeparator) {
        if (this.items.length > index) {
            this.items.splice(index, 0, item);
            if(item instanceof MenuItem) {
                item.render().addEventListener('click', () => {
                    item.callback(this);
                });
            }
        } else {
            // if the index is larger the the size of the menu items array
            // simply add it using addItem
            this.addItem(item);
        }
    } else {
        return TypeError;
    }
}

Menu .prototype .removeItem = function (item) {
    if (item instanceof MenuItem || item instanceof MenuSeparator) {
        let index;
        for (index in this.items) {
            if (this.items[index] === item) {
                this.items.splice(index, 1);
            }
        }
    } else {
        return TypeError;
    }
}

Menu .prototype .bodyClickCallback = (e, m) => {
    let t = e.target;
    if (!(t.matches('.popUpMenu') || t.matches('.menuItemLink')
        || t.matches('.menuItemDiv') || t.matches('.menuItemSeparator'))
        && m.bHidden === false) {
        // If something else then the Menu was clicked and the
        // menu is not hidden:
        // Stop the propagation of events and hide the menu
        e.stopPropagation();
        e.preventDefault();
        m.hide();
    }
}

Menu .prototype .show = function (id) {
    // create the DOM for the menu and append it to the body
    this.menuDiv = document.getElementById(id);

    // iterate trough items and append their rendered DOMs to the menu DOM
    let items_index
    for (items_index in this.items) {
        this.menuDiv.appendChild(this.items[items_index].render());
    }

    this.bHidden = false;
}

Menu .prototype .hide = function () {
    document.body.removeChild(this.menuDiv);
    this.bHidden = true;
}

function MenuItem (text, callback) {
    this.text = text;
    this.callback = callback;
    this.itemDiv = document.createElement("div");
    this.itemDiv.classList.add("menuItemDiv");
    this.linkElement = document.createElement("a");
    this.linkElement.classList.add("menuItemLink");
    this.itemDiv.appendChild(this.linkElement);
    this.linkElement.innerText = this.text;
    return this;
}

MenuItem .prototype .render = function () {
    return this.itemDiv;
}

function MenuSeparator () {
    this.separatorElement = document.createElement("hr");
    this.separatorElement.classList.add("menuItemSeparator");
    return this;
}

MenuSeparator .prototype .render = function () {
    return this.separatorElement;
}