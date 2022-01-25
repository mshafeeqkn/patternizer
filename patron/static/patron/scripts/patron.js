function dragElement(draggerId) {
    let draggerElement = $('#' + draggerId);
    let x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    let tableBody = $('#label-list > tbody');
    const draggerIndex = parseInt(draggerId.split("-").at(-1));
    const imageSize = $('#image').get(0).getBoundingClientRect();
    const draggerSize = draggerElement.get(0).getBoundingClientRect();

    const defTop = (imageSize.top + (imageSize.bottom - imageSize.top) / 2);
    const defLeft = (imageSize.left + (imageSize.right - imageSize.left) / 2);

    draggerElement.css({'top': defTop + "px", 'left': defLeft + "px"});

    $('#' + draggerId + "-handle").get(0).onmousedown = dragMouseDown;
    registerApplyButton();
    updatePositionInTable(tableBody.find('tr').eq(draggerIndex), defTop, defLeft);

    function dragMouseDown(e) {
        e = e || window.event;
        e.preventDefault();
        // get the mouse cursor position at startup:
        x2 = e.clientX;
        y2 = e.clientY;
        document.onmouseup = closeDragElement;
        // call a function whenever the cursor moves:
        document.onmousemove = elementDrag;
        $('body').css('cursor', 'none');
    }

    function elementDrag(e) {
        e = e || window.event;
        e.preventDefault();
        // calculate the new cursor position:
        x1 = x2 - e.clientX;
        y1 = y2 - e.clientY;
        x2 = e.clientX;
        y2 = e.clientY;

        // set the element's new position:
        const top = (draggerElement.get(0).offsetTop - y1);
        if (top > imageSize.top && top < imageSize.bottom - (draggerSize.bottom - draggerSize.top))
            draggerElement.get(0).style.top = top + "px";

        const left = (draggerElement.get(0).offsetLeft - x1);
        if (left > imageSize.left && left < imageSize.right - (draggerSize.right - draggerSize.left))
            draggerElement.get(0).style.left = left + "px";

        updatePositionInTable(tableBody.find('tr').eq(draggerIndex));
    }

    function updatePositionInTable(row) {
        let labelBox = $('#drag-text-' + draggerIndex).get(0).getBoundingClientRect();
        row.find('td').eq(2).text((labelBox.left - imageSize.left) + '');
        row.find('td').eq(3).text((imageSize.bottom - labelBox.bottom) + '');
    }

    function closeDragElement() {
        /* stop moving when mouse button is released:*/
        document.onmouseup = null;
        document.onmousemove = null;
        $('body').css('cursor', '');
    }

    function registerApplyButton() {
        $('#apply-btn-' + draggerIndex).on('click', function () {
            const btnSize = $(this).get(0).getBoundingClientRect();
            const dragItem = $('#' + draggerId);
            const draggerSize = dragItem.get(0).getBoundingClientRect();

            $(this).hide();
            $('#drag-item-' + draggerIndex + '-handle').hide();

            dragItem.css(
                {
                    'background-color': "rgba(255, 255, 255, 0)",
                    'top': draggerSize.top + (btnSize.bottom - btnSize.top - 2) + 'px'
                });
        });
    }
}

$(document).ready(function () {
    $('#add-label').on('click', function () {
        addNewLabel();
    });
});

let counter = 0;

function addNewLabel() {
    let labelField = $('#text-field');
    let label = labelField.val();
    let textLabel = '<div class="drag-item" id="drag-item-' + counter + '">' +
        '<div>' +
            '<span class="drag-btn" id="drag-item-' + counter + '-handle"><i class="fa fa-arrows text-success" aria-hidden="true"></i></span>' +
            '<span class="apply-btn" id="apply-btn-' + counter + '"><i class="fa fa-check text-success" aria-hidden="true"></i></span>' +
        '</div>' +
        '<div class="drag-text" id="drag-text-' + counter + '">' + label + '</div>' +
    '</div>';
    $('.container').append(textLabel);

    labelField.val("");

    if(counter === 0)
        $("#label-list > tbody").empty();

    $('#label-list > tbody').append('<tr><td>' + (counter + 1) + '</td><td>' + label + '</td><td></td><td></td></tr>');
    dragElement("drag-item-" + counter);

    counter++;
}


$('#text-field').val('Mohammed Shafeeque K N');
addNewLabel();
$('#text-field').val('Abdulla K N');
addNewLabel();
$('#text-field').val('9895653263');
addNewLabel();
