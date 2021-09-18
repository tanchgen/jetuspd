#ifndef _LIST_H
#define _LIST_H

#include <stddef.h>
#include <stdbool.h>
#include "stm32l1xx.h"


struct list_head {
	struct list_head  *next;
	struct list_head  *prev;
};

#define LIST_HEAD_INIT(name)	{&(name), &(name)}

#define LIST_HEAD(name) \
	struct list_head  name = LIST_HEAD_INIT(name)

/**
  * @brief	Initialize an entry.
  * 
  * @param[in]	list	the element to initialize
  *
  * @retval	none
  */
__STATIC_INLINE void INIT_LIST_HEAD(struct list_head *list)
{
	list->next = list;
	list->prev = list;
}

__STATIC_INLINE void __list_add(struct list_head *_new, struct list_head *prev, struct list_head *next)
{
	next->prev = _new;
	_new->next = next;
	_new->prev = prev;
	prev->next = _new;
}

/**
  * @brief	Add a new entry to the list (insert a new entry after the specified head).
  *
  * @note	This is good for implementing stacks.
  *
  * @param[in]	_new	the new entry to be added
  * @param[in]	head	the list head to add it after
  *
  * @retval	none
  */
__STATIC_INLINE void list_add(struct list_head *_new, struct list_head *head)
{
	__list_add(_new, head, head->next);
}

/**
  * @brief	Add a new entry to the list (insert a new entry before the specified head).
  *
  * @note	This is useful for implementing queues.
  *
  * @param[in]	_new	the new entry to be added
  * @param[in]	head	list head to add it before
  *
  * @retval	none
  */
__STATIC_INLINE void list_add_tail(struct list_head *_new, struct list_head *head)
{
	__list_add(_new, head->prev, head);
}

/**
  * @brief	Delete an entry from the list.
  *
  * @param[in]	entry	the element to delete from the list
  *
  * @retval	none
  */
__STATIC_INLINE void list_del(struct list_head *entry)
{
	entry->next->prev = entry->prev;
	entry->prev->next = entry->next;
}

/**
  * @brief	Replace an old entry by a new one.
  *
  * @note	If @old was empty, it will be overwritten.
  *
  * @param[in]	old	the element to be replaced
  * @param[in]	_new	the new element to insert
  *
  * @retval	none
  */
__STATIC_INLINE void list_replace(struct list_head *old, struct list_head *_new)
{
	_new->next = old->next;
	_new->next->prev = _new;
	_new->prev = old->prev;
	_new->prev->next = _new;
}

/**
  * @brief	Delete from one list and add as another's head.
  *
  * @param[in]	list	the entry to move
  * @param[in]	head	the head that will precede our entry
  *
  * @retval	none
  */
__STATIC_INLINE void list_move(struct list_head *list, struct list_head *head)
{
	list_del(list);
	list_add(list, head);
}

/**
  * @brief	Delete from one list and add as another's tail.
  *
  * @param[in]	list	the entry to move
  * @param[in]	head	the head that will follow our entry
  *
  * @retval	none
  */
__STATIC_INLINE void list_move_tail(struct list_head *list, struct list_head *head)
{
	list_del(list);
	list_add_tail(list, head);
}

/**
  * @brief	Tests whether @list is the last entry in list @head.
  *
  * @param[in]	list	the entry to test
  * @param[in]	head	the head of the list
  *
  * @retval	boolean true/false
  */
__STATIC_INLINE bool list_is_last(const struct list_head *list, const struct list_head *head)
{
	return (list->next == head);
}

/**
  * @brief	Tests whether a list is empty.
  *
  * @param[in]	head	the list to test
  *
  * @retval	boolean true/false
  */
__STATIC_INLINE bool list_empty(const struct list_head *head)
{
	return (head->next == head);
}

__STATIC_INLINE void __list_splice(const struct list_head *list, struct list_head *prev, struct list_head *next)
{
	struct list_head  *first, *last;

	first = list->next;
	last  = list->prev;

	first->prev = prev;
	prev->next = first;

	last->next = next;
	next->prev = last;
}

/**
  * @brief	Join two lists, this is designed for stacks.
  *
  * @param[in]	list	the new list to add
  * @param[in]	head	the place to add it in the first list
  *
  * @retval	none
  */
__STATIC_INLINE void list_splice(struct list_head *list, struct list_head *head)
{
	if (!list_empty(list))
		__list_splice(list, head, head->next);
}

/**
  * @brief	Join two lists, each list being a queue.
  *
  * @param[in]	list	the new list to add
  * @param[in]	head	the place to add it in the first list
  *
  * @retval	none
  */
__STATIC_INLINE void list_splice_tail(struct list_head *list, struct list_head *head)
{
	if (!list_empty(list))
		__list_splice(list, head->prev, head);
}

/**
  * @brief	Get the struct for this entry.
  *
  * @param[in]	ptr	the &struct list_head pointer
  * @param[in]	type	the type of the struct this is embedded in
  * @param[in]	member	the name of the list_struct within the struct
  *
  * @retval	the struct for the entry
  */
#define list_entry(ptr, type, member) \
	((type *)((size_t)(ptr) - offsetof(type, member)))

/**
  * @brief	Get the first element from a list.
  *
  * @note	The list is expected to be not empty.
  *
  * @param[in]	ptr	the &struct list_head pointer
  * @param[in]	type	the type of the struct this is embedded in
  * @param[in]	member	the name of the list_struct within the struct
  *
  * @retval	the first element of the list
  */
#define list_first_entry(ptr, type, member) \
	list_entry((ptr)->next, type, member)

/**
  * @brief	Iterate over a list.
  *
  * @param[out]	pos	the &struct list_head to use as a loop cursor
  * @param[in]	head	the head of the list
  *
  * @retval	none
  */
#define list_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)

/**
  * @brief	Iterate over a list backwards.
  *
  * @param[out]	pos	the &struct list_head to use as a loop cursor
  * @param[in]	head	the head of the list
  *
  * @retval	none
  */
#define list_for_each_reverse(pos, head) \
	for (pos = (head)->prev; pos != (head); pos = pos->prev)

/**
  * @brief	Iterate over a list safe against removal of a list entry.
  *
  * @param[out]	pos	the &struct list_head to use as a loop cursor
  * @param[in]	n	another &struct list_head to use as a temporary storage
  * @param[in]	head	the head of the list
  *
  * @retval	none
  */
#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); pos = n, n = pos->next)

/**
  * @brief	Iterate over a list backwards safe against removal of a list entry.
  *
  * @param[out]	pos	the &struct list_head to use as a loop cursor
  * @param[in]	n	another &struct list_head to use as a temporary storage
  * @param[in]	head	the head of the list
  *
  * @retval	none
  */
#define list_for_each_reverse_safe(pos, n, head) \
	for (pos = (head)->prev, n = pos->prev; pos != (head); pos = n, n = pos->prev)

/**
  * @brief	Count the number of items in a list.
  *
  * @param[in]	head	the head of the list
  *
  * @retval	length of the list
  */
__STATIC_INLINE size_t list_length(struct list_head *head)
{
	size_t             length;
	struct list_head  *item;

	length = 0;

	list_for_each(item, head)
		++length;

	return length;
}

#endif /* _LIST_H */

