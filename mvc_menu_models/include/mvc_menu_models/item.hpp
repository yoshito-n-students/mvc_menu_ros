#ifndef MVC_MENU_MODELS_ITEM_HPP
#define MVC_MENU_MODELS_ITEM_HPP

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <mvc_menu_models/xml_element.hpp>
#include <ros/console.h>

namespace mvc_menu_models {

// pointer types

class Item;
typedef std::shared_ptr< Item > ItemPtr;
typedef std::shared_ptr< const Item > ItemConstPtr;

// menu item.
// state of Item cannot be changed after construction by itemsFromDescription()
// because all methods of Item is const.

class Item : public std::enable_shared_from_this< Item > {
public:
  enum DisplayType { Name, AltTxt, Image };

protected:
  Item() {}

public:
  virtual ~Item() {}

  // propaties

  std::int32_t itemId() const { return item_id_; }

  const std::string &name() const { return name_; }

  std::string path(const char separator = '.') const {
    const ItemConstPtr p(parent());
    return p ? (p->path() + separator + name_) : name_;
  }

  DisplayType displayType() const { return display_type_; }

  const std::string &altTxt() const { return alt_txt_; }

  const std::string &imgURL() const { return img_url_; }

  // root

  ItemConstPtr root() const {
    const ItemConstPtr p(parent());
    return p ? p->root() : shared_from_this();
  }

  // parent

  ItemConstPtr parent() const { return parent_.lock(); }

  ItemConstPtr parentLevel() const {
    const ItemConstPtr p(parent());
    return p ? p->sibilingLevel() : ItemConstPtr();
  }

  // sibilings

  int row() const { return row_; }

  int col() const { return col_; }

  int numSibilings() const {
    const ItemConstPtr p(parent());
    return p ? p->children_.size() : 1;
  }

  std::vector< ItemConstPtr > sibilings() const {
    const ItemConstPtr p(parent());
    return p ? p->children_ : std::vector< ItemConstPtr >(1, shared_from_this());
  }

  ItemConstPtr sibiling(const int sid) const {
    const ItemConstPtr p(parent());
    if (p && sid >= 0 && sid < p->children_.size()) {
      return p->children_[sid];
    } else if (!p && sid == 0) {
      return shared_from_this();
    } else {
      return ItemConstPtr();
    }
  }

  ItemConstPtr sibiling(const int row, const int col) const {
    const ItemConstPtr p(parent());
    if (p && row >= 0 && row < p->rows_ && col >= 0 && col < p->cols_) {
      return p->children_[row * p->cols_ + col];
    } else if (!p && row == row_ && col == col_) {
      return shared_from_this();
    } else {
      return ItemConstPtr();
    }
  }

  ItemConstPtr sibilingLevel() const {
    const ItemConstPtr p(parent());
    if (p) {
      for (const ItemConstPtr &s : p->children_) {
        if (s) {
          return s;
        }
      }
    }
    return shared_from_this();
  }

  int depth() const {
    const ItemConstPtr p(parent());
    return p ? p->depth() + 1 : 0;
  }

  // children

  int rows() const { return rows_; }

  int cols() const { return cols_; }

  int numChildren() const { return children_.size(); }

  const std::vector< ItemConstPtr > &children() const { return children_; }

  ItemConstPtr child(const int cid) const {
    return (cid >= 0 && cid < children_.size()) ? children_[cid] : ItemConstPtr();
  }

  ItemConstPtr child(const int row, const int col) const {
    return (row >= 0 && row < rows_ && col >= 0 && col < cols_) ? children_[row * cols_ + col]
                                                                : ItemConstPtr();
  }

  ItemConstPtr childLevel() const {
    for (const ItemConstPtr &c : children_) {
      if (c) {
        return c;
      }
    }
    return ItemConstPtr();
  }

  // factory

  static std::vector< ItemConstPtr > itemsFromDescription(const std::string &desc) {
    struct Internal {
      static bool appendItems(const XmlElement &elm, std::vector< ItemConstPtr > *const items,
                              const ItemPtr &parent_item = ItemPtr(), const int default_row = 0) {
        // is the element name "item"?
        if (elm.name() != "item") {
          ROS_ERROR_STREAM("Item::itemsFromDescription(): Unexpected element '" << elm.name()
                                                                                << "'");
          return false;
        }

        // create an item and append it to the given list
        const ItemPtr item(new Item());
        item->item_id_ = items->size();
        items->push_back(item);

        // set row & col indice
        const int row(elm.attribute("row", default_row)), col(elm.attribute("col", 0));
        if (row < 0 || col < 0) {
          ROS_ERROR_STREAM("Item::itemsFromDescription(): (" << row << ", " << col
                                                             << ") is out of range");
          return false;
        }
        item->row_ = row;
        item->col_ = col;

        // associate the item with the parent
        if (parent_item) {
          const int cid(row * parent_item->cols_ + col);
          if (cid < 0 || cid >= parent_item->children_.size()) {
            ROS_ERROR_STREAM("Item::itemsFromDescription(): child (" << row << ", " << col
                                                                     << ") is out of range");
            return false;
          } else if (parent_item->children_[cid]) {
            ROS_ERROR_STREAM("Item::itemsFromDescription(): Multiple child items in ("
                             << row << ", " << col << ")");
            return false;
          }
          parent_item->children_[cid] = item;
          item->parent_ = parent_item;
        }

        // load the item name from the attribute
        if (!elm.getAttribute("name", &item->name_)) {
          ROS_ERROR("Item::itemsFromDescription(): No attribute 'name'");
          return false;
        }

        // load the display type from the attribute
        const std::string display(elm.attribute< std::string >("display", "name"));
        if (display == "name") {
          item->display_type_ = Item::Name;
        } else if (display == "alttxt") {
          item->display_type_ = Item::AltTxt;
          if (!elm.getAttribute("alttxt", &item->alt_txt_)) {
            ROS_ERROR("Item::itemsFromDescription(): No attribute 'alttxt'");
            return false;
          }
        } else if (display == "image") {
          item->display_type_ = Item::Image;
          if (!elm.getAttribute("imgurl", &item->img_url_)) {
            ROS_ERROR("Item::itemsFromDescription(): No attribute 'imgurl'");
            return false;
          }
        } else {
          ROS_ERROR_STREAM("Item::itemsFromDescription(): Unknown display type '" << display
                                                                                  << "'");
          return false;
        }

        // allocate child items
        const int rows(elm.attribute("rows", elm.numChildElements())),
            cols(elm.attribute("cols", 1));
        if (rows < 0 || cols < 0) {
          ROS_ERROR_STREAM("Item::itemsFromDescription(): Invalid child size (" << rows << ", "
                                                                                << cols << ")");
          return false;
        }
        item->rows_ = rows;
        item->cols_ = cols;
        item->children_.resize(rows * cols);

        // recursively update the given list
        const std::vector< XmlElementConstPtr > child_elms(elm.childElements());
        for (std::size_t i = 0; i < child_elms.size(); ++i) {
          if (!appendItems(*child_elms[i], items, item, i)) {
            return false;
          }
        }

        return true;
      }
    };

    // parse the given xml description
    const XmlElementConstPtr elm(XmlElement::fromString(desc));
    if (!elm) {
      return std::vector< ItemConstPtr >();
    }

    // populate items by parsing the root xml element
    std::vector< ItemConstPtr > items;
    if (!Internal::appendItems(*elm, &items)) {
      return std::vector< ItemConstPtr >();
    }

    return items;
  }

protected:
  typedef std::weak_ptr< const Item > ItemWeakConstPtr;

  std::int32_t item_id_; // int32_t is the type of ids in State msg
  std::string name_;
  DisplayType display_type_;
  std::string alt_txt_, img_url_;
  ItemWeakConstPtr parent_;
  int row_, col_, rows_, cols_;
  std::vector< ItemConstPtr > children_;
};
} // namespace mvc_menu_models

#endif